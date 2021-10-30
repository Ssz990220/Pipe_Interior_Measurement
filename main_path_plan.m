%% Clean up
clc
clear
close all
%% create parpool 
if isempty(gcp('nocreate'))
    parpool
end
%% Load env & Env
% [robot, collision_obj, ax, start, target] = setupBoxEnv();
% [robot, collision_obj, ax, start, target] = setupPipeEnv();
[robot, collision_obj, ax, start, target] = setupTStructureEnv();

%% Set up planner
% StateSpace and StateValidator
ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj);
%% GMM planner
options = GMM_RRT_T_Config(start, target);
% options = GMM_RRT_Pipe_Config(start, target);
% options = GMM_RRT_Box_Config(start, target);
% Setup
planner = plannerGMMRRT(ss,sv,options);
%% Time Saver
tic;
planner.init();
planner.GoalBias=0.2;
t = toc;
fprintf('Initilization took %.4f sec...\n',t);
%% GMM Plan
i=0;
time_GMM_RRT_plan = [];
time_GMM_RRT_update = [];
path_found = [];
kin_check=[];
Total_col_check=[];
for i = 1:10
tic;
[pathObj, solnInfo] = planner.plan(start, target);
t = toc;
time_GMM_RRT_plan = [time_GMM_RRT_plan,t];
fprintf('Path planning of iter %d took %.4f sec.\n',[i, t]);
% save Temp.mat
% %% Temp Test
% clear
% clc
% load Temp.mat
tic;
fprintf("Updating with %d ambigous states and %d incorrect states...\n",...
    [size(planner.StateValidator.ambigous_states_pool,1),size(planner.StateValidator.false_col_free_pose,1)]);
planner.update_GMM_model();
t = toc;
fprintf('Updating of iter %d took %.4f sec.\n',[i, t]);
fprintf('Planning iteration %d is done. PathFoundFlag = %d, ExitFlag = %d, \nKinematic Check %d times, Total Collision Check %d times\n',...
    [i,solnInfo.IsPathFound,solnInfo.ExitFlag, solnInfo.Kin_check,solnInfo.Total_col_check]);
path_found = [path_found,solnInfo.IsPathFound];
kin_check = [kin_check,solnInfo.Kin_check];
Total_col_check = [Total_col_check,solnInfo.Total_col_check];
if solnInfo.ExitFlag == 1
    q = pathObj.States;
    visualize_traj(robot, q, ax, 1, i);
    if solnInfo.IsPathFound
%         break
    end
end
end
%% GMM Bi RRT Plan
svBiRRT = UR10StateValidatorGMMBiRRT(ss,robot,collision_obj);
svBiRRT.GMM_col_model = planner.gmm_col_model_final;
svBiRRT.GMM_free_model = planner.gmm_free_model_final;
plannerGMMBiRRT = plannerBiRRT(ss,sv);
%% GMM BiRRT Plan
tic;
[pathObj, solnInfo] = plannerGMMBiRRT.plan(start, target)
toc
q = pathObj.States;
visualize_traj(robot, q, ax);
%% RRT Plan
tic;
sv.clean_counter();
planner = plannerRRT(ss,sv);
planner.GoalReachedFcn = @GMMGoalReachedFunction;
[pathObj, solnInfo] = planner.plan(start,target);
toc
%% Visualize the RRT result
q = pathObj.States;
visualize_traj(robot, q, ax);
%% BiRRT Plan
time_BiRRT_plan = [];
path_length_BiRRT = [];
% time_BiRRT_update = [];
% path_found_BiRRT = [];
kin_check_BiRRT=[];

planner = plannerBiRRT(ss,sv);
planner.MaxConnectionDistance = 0.08;
for i = 1:8
tic;
sv.clean_counter();
[pathObj, solnInfo] = planner.plan(start,target);
t = toc
time_BiRRT_plan = [time_BiRRT_plan,t];
% path_found_BiRRT = [path_found_BiRRT,solnInfo.IsPathFound];
kin_check_BiRRT = [kin_check_BiRRT,planner.StateValidator.kin_check_counter];
q = pathObj.States;
path_length_BiRRT = [path_length_BiRRT,size(q,1)];
% Visualize 
q = pathObj.States; 
visualize_traj(robot, q, ax,1,i);
end
