classdef plannerGMMRRT < plannerRRT & handle
    %PLANNERGMMRRT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant, Access = {?nav.algs.internal.InternalAccess})
        %ClassName
        ClassName = 'plannerGMMRRT'
    end
    properties
        gmm_rrt_options               %TODO
        num_init_sampler                % Number of points for initialization
        gmm_free_model                  % GMM model for free states
        gmm_col_model                   % GMM model for collision states
        num_collision_check             % Number of times of collision check
        is_init                         % init flag. Raise warning if not init.
        display_init_result             % display mahal distance after finishing initialization
        gmm_col_model_final
        gmm_free_model_final
    end
    
    methods
        function obj = plannerGMMRRT(ss,sv,options)
            %PLANNERGMMRRT Construct an instance of this classtreeInternal
            %   Detailed explanation goes here
            obj@plannerRRT(ss, sv);
            obj.num_init_sampler = options.num_init_sampler;
            obj.gmm_rrt_options = options.gmm_rrt_options;
            obj.GoalReachedFcn = @GMMGoalReachedFunction;
            obj.is_init = false;
            obj.display_init_result = options.display_init_result;
        end
        
        function [pathObj, solnInfo] = plan(obj, startState, goalState)
            %plan Plan a path between two states
            %   PATH = plan(PLANNER, STARTSTATE, GOALSTATE) tries to find  
            %   a valid path between STARTSTATE and GOALSTATE. The planning
            %   is carried out based on the underlying state space and state
            %   validator of PLANNER. The output, PATH, is returned as a 
            %   navpath object.
            %
            %   [PATH, SOLNINFO] = plan(PLANNER, ...) also returns a struct,
            %   SOLNINFO, as a second output that gives additional    
            %   details regarding the planning solution.
            %   SOLNINFO has the following fields:
            %
            %   IsPathFound:  Boolean indicating whether a path is found
            %
            %      ExitFlag:  A number indicating why planner terminates
            %                 1 - 'GoalReached'
            %                 2 - 'MaxIterationsReached'
            %                 3 - 'MaxNumNodesReached'
            %
            %      NumNodes:  Number of nodes in the search tree when
            %                 planner terminates (not counting the root
            %                 node).
            %
            % NumIterations:  Number of "extend" routines executed
            %
            %      TreeData:  A collection of explored states that reflects
            %                 the status of the search tree when planner
            %                 terminates. Note that nan values are inserted
            %                 as delimiters to separate each individual
            %                 edge.

            
            if coder.target('MATLAB')
                cleaner = onCleanup(@()obj.cleanUp);
            end
            
            if ~obj.is_init
                error("The planner has not been initialized, run 'planner.init' before plan");
            end
            % check whether start and goal states are valid
            if ~all(obj.StateValidator.isStateValid(startState)) || ~all(all(isfinite(startState)))
                coder.internal.error('nav:navalgs:plannerrrt:StartStateNotValid');
            end
            
            if ~all(obj.StateValidator.isStateValid(goalState)) || ~all(all(isfinite(goalState)))
                coder.internal.error('nav:navalgs:plannerrrt:GoalStateNotValid');
            end
            
            startState = nav.internal.validation.validateStateVector(startState, ...
                                                              obj.StateSpace.NumStateVariables, 'plan', 'startState');
            goalState = nav.internal.validation.validateStateVector(goalState, ...
                                                              obj.StateSpace.NumStateVariables, 'plan', 'goalState');
                                                          
            % Wipe the slate clean for the this execution
            obj.PathCostAtIteration = nan(obj.MaxIterations, 1);
            
            if obj.GoalReachedFcn(obj, startState, goalState)
                pathObj = navPath(obj.StateSpace, [startState; goalState]);
                solnInfo = struct();
                solnInfo.IsPathFound = true;
                solnInfo.ExitFlag = obj.GoalReached;
                solnInfo.NumNodes = 1;
                solnInfo.NumIterations = 0;
                solnInfo.TreeData = [startState;...
                    nan(1,obj.StateSpace.NumStateVariables); ...
                    startState;...
                    goalState;...
                    nan(1,obj.StateSpace.NumStateVariables)];
                obj.PathCostAtIteration = pathObj.pathLength;
                return;
            end

            obj.CurrentGoalState = goalState;
            tentativeGoalIds = [];
            
            treeInternal = nav.algs.internal.SearchTree(startState, obj.MaxNumTreeNodes);
            extendsReversely = true;
            treeInternal.setCustomizedStateSpace(obj.StateSpace, ~extendsReversely);
            
            
            obj.preLoopSetup(treeInternal);
            
            s = rng;
            rng(s); % should not let the maxIterations affect the goalBias result
            
            pathFound = false;
            statusCode = obj.Unassigned;
            numIterations = 0;
            for k = 1:obj.MaxIterations
                randState = obj.StateSpace.sampleGMM(obj.gmm_free_model);
                
                if rand() < obj.GoalBias
                    randState = obj.CurrentGoalState;
                end
                
                [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal);
                
                if statusCode == obj.GoalReached 
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId]; %#ok<AGROW>
                    numIterations = k;
                    break;
                end
                
                if statusCode == obj.GoalReachedButContinue
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId]; %#ok<AGROW>
                end

                % Record the minCost in this iteration.
                % When tentativeGoalIds is empty evaluatePathCosts also returns 
                % empty, and makes the expression min([nan]), which returns nan,
                % as expected for the "not reached goal" case.
                % When tentativeGoalIds is not empty, the expression takes
                % the form min([a b ...]) and assigns the min cost from root
                % amongst the tentative goals as the iteration cost.
                obj.PathCostAtIteration(k) = min([nan, ...
                    evaluatePathCosts(obj, treeInternal, tentativeGoalIds)]);

                if statusCode == obj.MaxNumTreeNodesReached
                    numIterations = k;
                    break;
                end
            end
            
            if numIterations == 0
                numIterations = obj.MaxIterations;
            end
            
            obj.PathCostAtIteration = obj.PathCostAtIteration(1:numIterations);
            
            treeData = treeInternal.inspect();
            numNodes = treeInternal.getNumNodes()-1;
            
            exitFlag = statusCode;            
            if statusCode >= obj.MotionInCollision
                exitFlag = obj.MaxIterationsReached;
            end
            
            if pathFound
                costBest = inf;
                idBest = -1;
                for j = 1:length(tentativeGoalIds)
                    nid = tentativeGoalIds(j);
                    c = treeInternal.getNodeCostFromRoot(nid);
                    if c < costBest
                        idBest = nid;
                        costBest = c;
                    end
                end

                % Record the best cost for the last iteration, this is
                % needed in case last iteration was the first time path was
                % found.
                obj.PathCostAtIteration(numIterations) = costBest;

                pathStates = treeInternal.tracebackToRoot(idBest);

                pathObj = navPath(obj.StateSpace, flip(pathStates')); 
            else
                pathObj = navPath(obj.StateSpace);
            end

            
            solnInfo = struct();
            solnInfo.IsPathFound = pathFound;
            solnInfo.ExitFlag = exitFlag;
            solnInfo.NumNodes = numNodes;
            solnInfo.NumIterations = numIterations;
            solnInfo.TreeData = treeData';
            

        end
        
        function obj = init(obj)
            obj.is_init = true;
            init_samples = obj.StateSpace.sampleUniform(obj.num_init_sampler);
            init_samples_col_flags = obj.StateValidator.isStateValid(init_samples);
            obj.gmm_col_model = GMM_K_means(cvt_2n_space(init_samples(init_samples_col_flags,:)),10,obj.gmm_rrt_options);
            obj.gmm_col_model.BUILD_GMM();
            obj.gmm_col_model_final = obj.gmm_col_model.gmm_final;
            obj.gmm_free_model = GMM_K_means(cvt_2n_space(init_samples(~init_samples_col_flags,:)),10,obj.gmm_rrt_options);
            obj.gmm_free_model.BUILD_GMM();
            obj.gmm_free_model_final = obj.gmm_free_model.gmm_final;
            if obj.display_init_result
                col_dis = hybrid_dis(init_samples(init_samples_col_flags,:),obj.gmm_col_model_final, obj.gmm_free_model_final);
                free_dis = hybrid_dis(init_samples(~init_samples_col_flags,:),obj.gmm_col_model_final, obj.gmm_free_model_final);
                h1 = histogram(col_dis);
                h1.BinWidth = 2;
                h1.Normalization = 'probability';
                hold on
                h2 = histogram(free_dis);
                h2.Normalization = 'probability';
                h2.BinWidth = 2;
                hold off
            end
        end
    end
    methods (Access = {?nav.algs.internal.InternalAccess})
        
        
        
    end
end