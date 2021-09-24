function  meshes = pipe_loader(path,displacement)
    %PIPE_LOADER load the pipe as collision object into the robot env
    %   path: leads to the folder that all the convec pieces of the pipe
    %   are.
    %   displacement: SE3 matrix for displacement
    testfiledir = path;
    matfiles = dir(fullfile(testfiledir, '*.txt'));
    nfiles = length(matfiles);
    data  = cell(nfiles,1);
    for i = 1 : nfiles
       data{i}=importdata( fullfile(testfiledir, matfiles(i).name));
    end
    meshes = displace_meshes(data,displacement);
end

function meshes_ = displace_meshes(meshes, displacement)
    meshes_ = cell(size(meshes));
    for i =1:size(meshes,1)
        meshes_{i}=displace_mesh(meshes{i},displacement);
    end
end

function mesh_ = displace_mesh(mesh, displacement)
    mesh_homo = [mesh, ones(size(mesh,1),1)];
    mesh_ = zeros(size(mesh_homo,1),3);
    for i = 1: size(mesh_homo,1)
        new_point =(displacement * mesh_homo(i,:)')';
        mesh_(i,:) = new_point(1:3);
    end
end