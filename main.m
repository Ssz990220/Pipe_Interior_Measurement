%% Clean up
clc
clear
close all
%% create parpool 
if isempty(gcp('nocreate'))
    parpool
end
%% Load env & Env
[robot, collision_obj, ax, start, target] = setupBoxEnv();
% [robot, collision_obj, ax, start, target] = setupPipeEnv();

%% Set up planner
% StateSpace and StateValidator
ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj);
%% GMM planner
options = GMM_RRT_Config(start, target);
% Setup
planner = plannerGMMRRT(ss,sv,options);
tic;
planner.init();
t = toc;
fprintf('Initilization took %.4f sec...\n',t);
%% GMM Plan
for i = 1:5
tic;
[pathObj, solnInfo] = planner.plan(start, target);
t = toc;
fprintf('Path planning of iter %d took %.4f sec.\n',[i, t]);
% save Temp.mat
% %% Temp Test
% clear
% clc
% load Temp.mat
tic;
planner.update_GMM_model();
t = toc;
fprintf('Updating of iter %d took %.4f sec.\n',[i, t]);
if solnInfo.IsPathFound
    q = pathObj.States;
    visualize_traj(robot, q, ax);
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
tic;
sv.clean_counter();
planner = plannerBiRRT(ss,sv);
[pathObj, solnInfo] = planner.plan(start,target)
toc

%% Visualize 
q = pathObj.States;
visualize_traj(robot, q, ax);