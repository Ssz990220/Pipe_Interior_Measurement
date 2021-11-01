function [robot, collision_obj, ax, start, target] = setupTStructureEnv()
%SETUPGONGENV Summary of this function goes here
%   Detailed explanation goes here
% Obstacles
ground = collisionBox(5,5,0.02);
ground.Pose = trvec2tform([0,0,-0.013]);
box1 = collisionBox(1,0.1,0.4);
box1.Pose = trvec2tform([0.9,0,0.2]);
box2 = collisionBox(1,0.3,0.1);
box2.Pose = trvec2tform([0.9,0.2,0.4]);
box3 = collisionBox(1,0.3,0.1);
box3.Pose = trvec2tform([0.9,-0.2,0.4]);
collision_obj{1} = ground;
collision_obj{2} = box1;
collision_obj{3} = box2;
collision_obj{4} = box3;
figure;
hold all
axis equal
show(box1);
show(box2);
show(box3);
ax = gca;
robot = importrobot('universalUR10.urdf',"MeshPath",["../asset/ur_description/ur10/collision","../asset/ur_description/ur10/visual"]);
robot.DataFormat = 'row';

start = [-1,-1.25,2,-0.8,0,0];
target = [0,-1.2,1,-2,-2,0];
show(robot,start,'Parent',ax,"PreservePlot",true,"Collision","on","Visuals","off");
show(robot,target,'Parent',ax,"PreservePlot",true,"Collision","on","Visuals","off");
view(10,15);
xlim([-0.2,1.5])
ylim([-1,1])
zlim([0,1])
end

