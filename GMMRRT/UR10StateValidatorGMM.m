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
    end
    
    methods
        function obj = UR10StateValidatorGMM(StateSpace, robot, collision_obj, col_threshold, col_free_threshold)
            %UR10STATEVALIDATORGMM Construct an instance of this class
            %   Detailed explanation goes here
            obj@nav.StateValidator(StateSpace);
            obj.robot = robot;
            obj.collision_obj = collision_obj;
            obj.num_kin_check = 0;
            obj.ValidationDistance = 0.01;
            obj.col_free_threshold = col_free_threshold;
            obj.col_threshold = col_threshold;
        end
        
        function inCollision = isStateValid(obj, state, count)
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
                [inCollision,~,~]=checkCollision(robot, state, collision_obj,"IgnoreSelfCollision","on","Exhaustive","on");
                if count
                        num_kin_check = num_kin_check + 1;
                end
            end
            obj.num_kin_check = num_kin_check;
        end
        
        function inCollision = isStateValid_GMM(obj, state, GMM_col_model, GMM_free_model)
            state_2n = cvt_2n_space(state);
            dis_col = min(mahal(GMM_col_model,state_2n));
            dis_free = min(mahal(GMM_free_model,state_2n));
            hybrid_dis = dis_col - dis_free;
            if hybrid_dis < obj.col_threshold
                inCollision = true;
                return
            elseif hybrid_dis > obj.col_free_threshold
                inCollision = false;
                return
            else
                inCollision = obj.isStateValid(state);
                return
            end
        end

        function [isValid, lastValidState] = isMotionValid(obj, state1, state2)
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);
            
            isValid = true;
            for i = 1: size(interpStates,1)
               
                interpSt = interpStates(i,:);
                
                if ~obj.isStateValid_GMM(interpSt)
                    isValid = false; 
                    break;
                end
            end
            lastValidState = inf;
        end
        
        function copyObj = copy(obj)
    
        end
    end
end

