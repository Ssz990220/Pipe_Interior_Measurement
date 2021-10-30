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
robot.Gravity = [0,0,-9.81];
load q.mat
n_q = size(q,1);
idx = 0:1/(n_q-1):1;
idx = idx';
q = [idx,q]';
phi = ones(1,6)*2;          % velocity limit
alpha = ones(1,6)*1.5;     % acceleration limit
mu = ones(1,6)*9;        % torque limit
%% Spline
n_points = 200;
[s, qs, qds, qdds] = spline_joint_function(q, n_points);
[ds, cs, gs] = get_dcg(robot,qs, qds, qdds);

%% Motion Planning
tic;
b = solve_optimal_speed(ds, cs, gs, qs, qds, qdds, s, phi, alpha ,mu);
toc;
%% Post Process
[t,tau, qd_t, qdd_t] = postprocess(ds, cs, gs, b, qs, qds, qdds, s);
%% Plot result
visualize_result(6, t, tau, qs(1:end-1,:), qd_t, qdd_t, s, phi, alpha, mu, b);
%% Visualize Traj
visTimeStep = 0.1;

% Low-fidelity robot model  
motionModel = jointSpaceMotionModel('RigidBodyTree', robot);

% Control robot to target trajectory points in simulation using low-fidelity model
initState = [qs(1,:)';qd_t(1,:)'];
targetStates = [qs,qd_t,qdd_t];    
tic;
[ts,robotStates] = ode15s(@(ts,state) TimeBasedStateInputs(motionModel, t, targetStates, ts, state), [t(1):visTimeStep:t(end)], initState);
toc;
%% Visualize simulation
for j=1:size(robotStates,1)
    show(robot, robotStates(j,1:6),'PreservePlot', false,"Parent",ax);
%     poseNow = getTransform(robot, robotStates(j,1:6)', endEffector);
%     plot3(poseNow(1,4), poseNow(2,4), poseNow(3,4),'b.','MarkerSize',20)
%     if isMovingObst
%         tNow = t(j);
%         aPoseObsNow = interp1(timeObs,aaPosesObs,tNow);
%         aMovObs.Pose = trvec2tform(aPoseObsNow);
%         h1.Matrix = aMovObs.Pose;
%         bPoseObsNow = interp1(timeObs,bPosesObs,tNow);
%         bMovObs.Pose = trvec2tform(bPoseObsNow);
%         h2.Matrix = bMovObs.Pose;
%     end
    drawnow;
end