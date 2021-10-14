clc;
clear;
load temp.mat
planner.randSampleProb = 0.5;
tic;
[pathObj, SolnInfo] = planner.plan(start, target)
toc

%% Visual
figHandle = figure;
ground = collisionBox(2,2,0.02);
ground.Pose = trvec2tform([0,0,-0.013]);
box = collisionBox(0.6,0.1,0.4);
box.Pose = trvec2tform([0.6,0,0.2]);

i=0;
collision_obj{i+1}=ground;
show(ground);
% Get axis properties and set hold
ax2 = gca;
hold all;

robot = importrobot('universalUR10.urdf',"MeshPath",["../asset/ur_description/ur10/collision","../asset/ur_description/ur10/visual"]);
robot.DataFormat = 'row';


collision_obj{i+2}=box;
show(box,"parent",ax2);
% Visualize the robot in its home configuration
start = [0.5,-1.25,2,-0.8,0,0];
show(robot,start,"Parent",ax2);

% Update the axis size
axis equal

% Loop through the other positions
q = pathObj.States;
for i = 1:length(q)
    show(robot,q(i,:),"Parent",ax2,"PreservePlot",false);
    
    % Update the figure    
    drawnow
end