function [robot, collision_obj, ax, start, target] = setupPipeEnv()
%SETUPBOXENV Setup robot environment with a boxy obstacle

% Obstacles
ground = collisionBox(5,5,0.02);
ground.Pose = trvec2tform([0,0,-0.013]);
% box = collisionBox(1,0.1,0.4);
% box.Pose = trvec2tform([0.9,0,0.2]);
pipes = pipe_loader('./assets/Pipe/pipe_mat',0.001*eye(4)*trvec2tform([250,500,0])*eul2tform([0,0,pi/2]));

% i=0;

figHandle = figure;         % Figure Handle
hold all;
axis equal
for i = 1:size(pipes,1)
    collision_obj{i} = collisionMesh(pipes{i});
end
collision_obj{i+1}=ground;
% collision_obj{i+2}=box;
% Visualize
% show(ground)
ax = gca;
for i = 1:size(pipes,1)
    show(collision_obj{i},"Parent", ax);
end
% show(box,"parent",ax);
%% Load robot
robot = importrobot('universalUR10.urdf',"MeshPath",["../asset/ur_description/ur10/collision","../asset/ur_description/ur10/visual"]);
robot.DataFormat = 'row';

start = [-0.05,-1.25,2,-0.8,0,0];
target = [1,-1.25,2,-0.8,1,0];
% show(robot,start,'Parent',ax,"PreservePlot",true,"Collision","on","Visuals","off");
% show(robot,target,'Parent',ax,"PreservePlot",true,"Collision","on","Visuals","off");
view(60,20);
end

