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
%% Set up planner
% StateSpace and StateValidator
ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj);
%% GMM planner
options = GMM_RRT_Config(start, target);
% Setup
planner = plannerGMMRRT(ss,sv,options);
planner.init();
%% GMM Plan
tic;
[pathObj, solnInfo] = planner.plan(start, target)
toc
q = pathObj.States;
visualize_traj(robot, q, ax);
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