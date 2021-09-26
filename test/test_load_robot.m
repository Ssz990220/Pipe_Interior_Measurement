clear;
clc;
close all;
robot = importrobot('universalUR10.urdf',"MeshPath",["../asset/ur_description/ur10/collision","../asset/ur_description/ur10/visual"]);
a = pipe_loader('./assets/Pipe/pipe_mat',0.001*eye(4)*trvec2tform([500,500,0])*eul2tform([0,0,pi/2]));
hold on;
axis equal
xlabel 'x'
ylabel 'y'
zlabel 'z'
for i = 1:size(a,1)
    c_mesh = collisionMesh(a{i});
    show(c_mesh);
end
% hold off;
show(robot,"Collisions","on","Visuals","off")