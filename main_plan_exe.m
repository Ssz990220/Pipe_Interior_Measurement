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
% [robot, collision_obj, ax, start, target] = setupTStructureEnv();
%% Plan
plannerBiRRT_in = manipulatorRRT(robot, collision_obj);
% plannerBiRRT_in.MaxConnectionDistance = 0.05;
tic;
path = plan(plannerBiRRT_in, start, target);
fprintf("Planning took %.4f sec...\n",toc);
visualize_traj(robot, path,ax,0,'');
%% Smoothing
% path_smooth = b_spline(path,0:1/99:1);
% % Visualization
% visualize_traj(robot, path_smooth, ax, 0,'');
%% Motion Plan
% Setup
q = path;
robot.Gravity = [0,0,-9.81];
% n_q = size(q,1);
% idx = 0:1/(n_q-1):1;
% idx = idx';
% q = [idx,q]';
% Parameters
phi = ones(1,6)*2;          % velocity limit
alpha = ones(1,6)*1.5;     % acceleration limit
mu = ones(1,6)*9;        % torque limit
% Spline
n_points = 500;
tic;
% [s, qs, qds, qdds] = spline_joint_function(q, n_points);

[qs,qds, qdds,~] = b_spline_drv(q',5,0:1/99:1);
s = 0:1/99:1;
qs = qs';
qds = qds';
qdds = qdds';
fprintf("Spline took %.4f sec...\n",toc);
tic;
[ds, cs, gs] = get_dcg(robot,qs, qds, qdds);
fprintf("Get DCG took %.4f sec...\n",toc);
% Motion Planning
tic;
b = solve_optimal_speed(ds, cs, gs, qs, qds, qdds, s, phi, alpha ,mu);
toc;
% Post Process
[t,tau, qd_t, qdd_t] = postprocess(ds, cs, gs, b, qs, qds, qdds, s);
%% Visualize Result
visualize_result(6, t, tau, qs(1:end-1,:), qd_t, qdd_t, s, phi, alpha, mu, b)
%% Prepare Visualization
visTimeStep = 0.1;

% Low-fidelity robot model  
motionModel = jointSpaceMotionModel('RigidBodyTree', robot);

% Control robot to target trajectory points in simulation using low-fidelity model
initState = [qs(1,:)';qd_t(1,:)'];
targetStates = [qs,qd_t,qdd_t];    
tic;
[ts,robotStates] = ode15s(@(ts,state) TimeBasedStateInputs(motionModel, t, targetStates, ts, state), [t(1):visTimeStep:t(end)], initState);
toc;
%% Visualization
% for j=1:size(robotStates,1)
%     show(robot, robotStates(j,1:6),'PreservePlot', false,"Parent",ax,'Collisions','on',"Visuals",);
%     drawnow;
% end
visualize_traj(robot, robotStates(:,1:6),ax,1,'');