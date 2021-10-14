%% Clean up
clc
clear
close all
%% create parpool 
if isempty(gcp('nocreate'))
    parpool
end
%% Load env & Env
[robot, collision_obj, ax] = setupBoxEnv();
%% Set up planner
start = [0.5,-1.25,2,-0.8,0,0];
target = [-1,-1.25,2,-0.8,1,0];
% StateSpace and StateValidator
ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj, -0.35,0.5);
%% GMM planner
options = GMM_RRT_Config(start, target);
% Setup
planner = plannerGMMRRT(ss,sv,options);
planner.init();
%%
% tester = planner_tester(planner);
% tester.init_tester();
%%
tic;
[pathObj, solnInfo] = planner.plan(start, target)
toc
% planner.update_GMM_model();                         % Online Learning
%% Visualize the result
q = pathObj.States;
visualize_traj(robot, q, ax);
%% GMM Bi RRT Plan
tic;
sv.clean_counter();
planner = plannerBiRRT(ss,sv);
%% RRT Plan
tic;
ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj, -0.35,0.5);

planner = plannerRRT(ss,sv);
planner.GoalReachedFcn = @GMMGoalReachedFunction;
[pathObj, solnInfo] = planner.plan(start,target);
toc
%% Visualize the RRT result
q = pathObj.States;
visualize_traj(robot, q, ax);
%% BiRRT Plan
tic;
ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj, -0.35,0.5);

planner = plannerBiRRT(ss,sv);
[pathObj, solnInfo] = planner.plan(start,target);
toc

%% Visualize 
q = pathObj.States;
visualize_traj(robot, q, ax);