classdef UR10StateValidatorGMM < nav.StateValidator & handle
    %UR10STATEVALIDATORGMM Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant, Access=protected)
        GMM_col_free = 1
        GMM_col = 2
        GMM_col_uncertain = 3
        kin_col_free = 4
        kin_col = 5
    end    
    properties
        robot
        collision_obj
        num_kin_check
        ValidationDistance
        col_threshold
        col_free_threshold
        ambigous_states_pool               % Restore ambigous states
        ambigous_states_flag_pool
        ax
        visual
        false_col_free_pose         % Restore false collision free states in final double check
    end
    
    methods
        function obj = UR10StateValidatorGMM(StateSpace, robot, collision_obj, col_threshold, col_free_threshold, ax)
            %UR10STATEVALIDATORGMM Construct an instance of this class
            %   Detailed explanation goes here
            obj@nav.StateValidator(StateSpace);
            obj.robot = robot;
            obj.collision_obj = collision_obj;
            obj.num_kin_check = 0;
            obj.ValidationDistance = 0.01;                          %
            obj.col_free_threshold = col_free_threshold;
            obj.col_threshold = col_threshold;
            if nargin == 6
                obj.ax = ax;
                obj.visual = true;
            end
        end
        
        function isValid = isStateValid(obj, state, count)
            if nargin == 2
                count = true;
            end
            inCollision = false(size(state,1), 1); % Check whether each pose is in collision
%             worldCollisionPairIdx = cell(size(state,1),1); % Provide the bodies that are in collision
            robot = obj.robot;
            collision_obj = obj.collision_obj;
            num_kin_check = obj.num_kin_check;
            if size(state,1) > 1
                parfor i = 1:size(state,1)
                   [inCollision(i),~,~]=checkCollision(robot, state(i,:), collision_obj,"IgnoreSelfCollision","on","Exhaustive","on");
    %                [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    %                 worldCollidingPairs = [bodyIdx,worldCollisionObjIdx]; 
    %                 worldCollisionPairIdx{i} = worldCollidingPairs;
                    if mod(i,100) == 0
                        fprintf('Collision check for %d is complete\n',i);
                    end
                    if count
                        num_kin_check = num_kin_check + 1;
                    end
                end
            else
                [inCollision,sepDist,~]=checkCollision(robot, state, collision_obj,"IgnoreSelfCollision","on","Exhaustive","on");
                if obj.visual & inCollision
                    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
                    HighlightCollisionBodies(robot,bodyIdx,obj.ax);
                end
                if count
                        num_kin_check = num_kin_check + 1;
                end
            end
            obj.num_kin_check = num_kin_check;
            isValid = ~inCollision;
        end
        
        function isValid = isStateValid_GMM(obj, state, GMM_col_model, GMM_free_model)
            state_2n = cvt_2n_space(state);
            dis_col = min(mahal(GMM_col_model,state_2n),[],2);
            dis_free = min(mahal(GMM_free_model,state_2n),[],2);
            hybrid_dis = dis_col - dis_free;
            isValid = false(size(state,1),1);
            for i = 1:size(state,1)
                if hybrid_dis(i) < obj.col_threshold
                    isValid(i) = false;
                elseif hybrid_dis(i) > obj.col_free_threshold
                    isValid(i) = true;
                else
                    isValid(i) = obj.isStateValid(state(i,:));
                    obj.add_ambigous_pose(state(i,:), isValid(i));
                end
            end
            
        end
        
        function [isValid, lastValidState] = isMotionValid(obj, state1, state2, final_check)
            if nargin == 3
                final_check = false;
            end
            
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);
            
            isValid = true;
            for i = 1: size(interpStates,1)
               
                interpSt = interpStates(i,:);
                
                if ~obj.isStateValid(interpSt)
                    isValid = false; 
                    if final_check
                        obj.add_false_col_free_pose(interpSt);
                    end
                    break;
                end
            end
            lastValidState = inf;
        end
        
        function [isValid, lastValidState] = isMotionValid_GMM(obj, state1, state2, GMM_col_model, GMM_free_model)
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);
            
            isValid = true;
            for i = 1: size(interpStates,1)
               
                interpSt = interpStates(i,:);
                
                if ~obj.isStateValid_GMM(interpSt, GMM_col_model, GMM_free_model)
                    isValid = false; 
                    break;
                end
            end
            lastValidState = inf;
        end
        
        function isValid = isTrajValid(obj,states)
           traj_len = size(states,1);
           isValid_sec = false(traj_len-1,1);
           for i = 1:traj_len-1
               isValid_sec(i) = obj.isMotionValid(states(i,:),states(i+1,:),true);          % final check
           end
           isValid = ~any(~isValid_sec);
        end
        
        function copyObj = copy(obj)
    
        end
        
        function obj = add_ambigous_pose(obj, state, flag)
           obj.ambigous_states_pool = [obj.ambigous_states_pool;state];
           obj.ambigous_states_flag_pool = [obj.ambigous_states_flag_pool, flag];
        end
        
        function obj = clean_ambigous_pose_pool(obj)
           obj.ambigous_states_pool = [];
           obj.ambigous_states_flag_pool = [];
        end
        
        function obj = add_false_col_free_pose(obj, state)
           obj.false_col_free_pose = [obj.false_col_free_pose; state];
        end
        
        function obj = clean_false_col_free_pose(obj)
           obj.false_col_free_pose = []; 
        end
    end
end

