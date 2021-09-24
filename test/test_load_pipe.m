clear
clc
a = pipe_loader('./assets/Pipe/pipe_mat',eye(4));
hold on;
axis equal
xlabel 'x'
ylabel 'y'
zlabel 'z'
for i = 1:size(a,1)
    c_mesh = collisionMesh(a{i});
    show(c_mesh);
end
