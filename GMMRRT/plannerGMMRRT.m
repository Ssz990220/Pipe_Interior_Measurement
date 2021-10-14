classdef plannerGMMRRT < plannerRRT & handle
    %PLANNERGMMRRT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant, Access = {?nav.algs.internal.InternalAccess})
        %ClassName
        ClassName = 'plannerGMMRRT'
    end
    properties
        gmm_rrt_col_options               %TODO
        gmm_rrt_free_options            
        num_init_sampler                % Number of points for initialization
        gmm_free_model                  % GMM model for free states
        gmm_col_model                   % GMM model for collision states
        num_collision_check             % Number of times of collision check
        is_init                         % init flag. Raise warning if not init.
        display_init_result             % display mahal distance after finishing initialization
        gmm_col_model_final
        gmm_free_model_final
        randSampleProb
        col_false_positive_prob         % Has to be small to avoid collision
        col_true_negative_prob          % Has to be high
        init_samples                    
        init_samples_col_flags
        trajectory_based_sample         % Sample type while init
        fixed_gmm                       % If or not use gmm with fixed number of components
        fixed_gmm_options
        
    end
    
    methods
        function obj = plannerGMMRRT(ss,sv,options)
            %PLANNERGMMRRT Construct an instance of this classtreeInternal
            %   Detailed explanation goes here
            obj@plannerRRT(ss, sv);
            obj.num_init_sampler = options.num_init_sampler;
            obj.gmm_rrt_col_options = options.gmm_rrt_col_options;
            obj.gmm_rrt_free_options = options.gmm_rrt_free_options;
            obj.randSampleProb = options.randSampleProb;
            obj.GoalReachedFcn = @GMMGoalReachedFunction;
            obj.is_init = false;
            obj.display_init_result = options.display_init_result;
            obj.col_false_positive_prob = options.col_false_positive_prob;
            obj.col_true_negative_prob = options.col_true_negative_prob;
            obj.trajectory_based_sample = options.trajectory_based_sample;
            obj.fixed_gmm = options.fixed_gmm;
            if obj.fixed_gmm
                obj.fixed_gmm_options = options.fixed_gmm_options;
            end
            
            if obj.trajectory_based_sample
                obj.init_samples = obj.StateSpace.sample_around_traj(options.start, options.target, options.var, obj.num_init_sampler);
            else
                obj.init_samples = obj.StateSpace.sampleUniform(obj.num_init_sampler);
            end
            obj.init_samples_col_flags = obj.StateValidator.isStateValid(obj.init_samples);
            obj.StateValidator.num_kin_check=0;
        end
        
        function [pathObj, solnInfo] = plan(obj, startState, goalState)
            if coder.target('MATLAB')
                cleaner = onCleanup(@()obj.cleanUp);
            end
            
            if ~obj.is_init
                error("The planner has not been initialized, run 'planner.init' before plan");
            end
            % check whether start and goal states are valid
            if ~all(obj.StateValidator.isStateValid_GMM(startState,obj.gmm_col_model_final, obj.gmm_free_model_final)) || ~all(all(isfinite(startState)))
                coder.internal.error('nav:navalgs:plannerrrt:StartStateNotValid');
            end
            
            if ~all(obj.StateValidator.isStateValid_GMM(goalState,obj.gmm_col_model_final, obj.gmm_free_model_final)) || ~all(all(isfinite(goalState)))
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
            
            s = rng;
            rng(s); % should not let the maxIterations affect the goalBias result
            
            pathFound = false;
            statusCode = obj.Unassigned;
            numIterations = 0;
            for k = 1:obj.MaxIterations
                if rand() < obj.randSampleProb
                    randState = obj.StateSpace.sampleUniform();
                else
                    randState = obj.StateSpace.sampleGMM(obj.gmm_free_model_final);
                end
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
                % Double Check 
                pathStates = pathObj.States;
                pathFound = obj.StateValidator.isTrajValid(pathStates);
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
            init_samples = obj.init_samples;
            init_samples_col_flags = obj.init_samples_col_flags;
            if obj.fixed_gmm
                obj.gmm_col_model_final = fitgmdist(cvt_2n_space(init_samples(init_samples_col_flags,:)),obj.fixed_gmm_options.num_component);
                obj.gmm_free_model_final = fitgmdist(cvt_2n_space(init_samples(~init_samples_col_flags,:)),obj.fixed_gmm_options.num_component);
            else
                obj.gmm_col_model = GMM_K_means(cvt_2n_space(init_samples(init_samples_col_flags,:)),10,obj.gmm_rrt_col_options);
                obj.gmm_col_model.BUILD_GMM();
                obj.gmm_col_model_final = obj.generate_final_gmm(obj.gmm_col_model.gmm_final);
                obj.gmm_free_model = GMM_K_means(cvt_2n_space(init_samples(~init_samples_col_flags,:)),10,obj.gmm_rrt_free_options);
                obj.gmm_free_model.BUILD_GMM();
                obj.gmm_free_model_final = obj.generate_final_gmm(obj.gmm_free_model.gmm_final);
            end
            if obj.display_init_result
                col_dis = hybrid_dis(init_samples(init_samples_col_flags,:),obj.gmm_col_model_final, obj.gmm_free_model_final);
                free_dis = hybrid_dis(init_samples(~init_samples_col_flags,:),obj.gmm_col_model_final, obj.gmm_free_model_final);
                % Fit Distribution
                col_dist = fitdist(col_dis','Normal');
                free_dist = fitdist(free_dis','Normal');
                % Threshold
                free_threshold = norminv(1-obj.col_false_positive_prob,col_dist.mu, col_dist.sigma);
                col_threshold = norminv(obj.col_true_negative_prob,col_dist.mu,col_dist.sigma);
                % 
%                 true_positive_prob = normcdf(free_threshold, free_dist.mu, free_dist.sigma,'upper');
%                 false_positive_prob = obj.col_false_positive_prob;
%                 true_negative_prob = obj.col_true_negative_prob;
%                 false_negative_prob = normcdf(col_threshold, free_dist.mu, free_dist.sigma);
                true_positive_prob = size(free_dis(free_dis>free_threshold),2)/size(free_dis,2);
                false_positive_prob = size(col_dis(col_dis>free_threshold),2)/size(col_dis,2);
                false_negative_prob = size(free_dis(free_dis<col_threshold),2)/size(free_dis,2);
                true_negative_prob = size(col_dis(col_dis<col_threshold),2)/size(col_dis,2);
                
                fprintf('\tPos\tNeg\nTrue%d%% %d%%\nFalse %d%% %d%% \n',...
                    [round(true_positive_prob*100),round(true_negative_prob*100),...
                    round(false_positive_prob*100),round(false_negative_prob*100)])
                
                % Histogram
                figure
                h1 = histogram(col_dis);
                xlim([-50,50]);
                ylim([0,0.05]);
                h1.BinWidth = 2;
                h1.Normalization = 'probability';
                hold on
                h2 = histogram(free_dis);
                h2.Normalization = 'probability';
                h2.BinWidth = 2;
                % Distribution plot
%                 Range_min = min(min(col_dis), min(free_dis)) * 1.5;
%                 Range_max = max(max(col_dis),max(free_dis))*1.5;
                x = -50:0.1:50;
                plot(x,normpdf(x,free_dist.mu, free_dist.sigma),'Color','r');
                plot(x,normpdf(x,col_dist.mu, col_dist.sigma),'Color','b');
                % Threshold Plot
                plot(ones(100,1)*col_threshold,(1:100)/1000,'--','Color','b');
                plot(ones(100,1)*free_threshold,(1:100)/1000,'--','Color','r');
                legend('Collision','Collision Free','Collision Free Distribution','Collision Distribution','Collision Threshold','Collision Free Threshold');
                hold off
            end
            
            obj.StateValidator.col_threshold = col_threshold;
            obj.StateValidator.col_free_threshold = free_threshold;
        end
        function obj = update_GMM_model(obj)
            obj.update_with_ambigous_points(obj);
        end
    end
    methods (Access = {?nav.algs.internal.InternalAccess})
       function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            %extend The RRT "Extend" routine
            statusCode = obj.InProgress;
            newNodeId = nan;
            idx = treeInternal.nearestNeighbor(randState);
            nnState = treeInternal.getNodeState(idx);
                
            d = obj.StateSpace.distance(randState, nnState);
            newState = randState;
            
            % steer
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nnState, randState, obj.MaxConnectionDistance/d);  % L/d*(randState - nnState) + nnState;
            end

            % check motion validity with GMM model
            if ~obj.StateValidator.isMotionValid_GMM(nnState, newState, obj.gmm_col_model_final, obj.gmm_free_model_final)
                statusCode = obj.MotionInCollision;
                return;
            end

            newNodeId = treeInternal.insertNode(newState, idx);
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                statusCode = obj.GoalReached;
                return;
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return
            end
            
       end 
        
       function obj = update_with_ambigous_points(obj)
           ambigous_states = obj.StateValidator.ambigous_states_pool;
           ambigous_states_flag = obj.StateValidator.ambigous_states_flag_pool;
           % TODO
       end
       
       function gm = generate_final_gmm(obj, gm_final)
          mu = gm_final.mu;
          sigma = gm_final.Sigma;
          n_components = gm_final.NumComponents;
          p = ones(1,n_components)*1/n_components;
          gm = gmdistribution(mu, sigma, p);
       end
    end
end