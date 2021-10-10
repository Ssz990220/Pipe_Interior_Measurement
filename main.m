%% Clean up
clc
clear
close all
%% parpool create
if isempty(gcp('nocreate'))
    parpool
end
%% Load robot & env
robot = importrobot('universalUR10.urdf',"MeshPath",["../asset/ur_description/ur10/collision","../asset/ur_description/ur10/visual"]);
robot.DataFormat = 'row';
pipes = pipe_loader('./assets/Pipe/pipe_mat',0.001*eye(4)*trvec2tform([500,500,0])*eul2tform([0,0,pi/2]));
for i = 1:size(pipes,1)
    collision_obj{i} = collisionMesh(pipes{i});
end

ss = UR10StateSpaceGMM;
sv = UR10StateValidatorGMM(ss, robot, collision_obj, -0.35,0.5);
options.num_init_sampler = 2000;
options.display_init_result = true;
gmm_rrt_options.max_iter = 20;
gmm_rrt_options.bhat_dis_threshold = 5;
gmm_rrt_options.start_merge_threshold = 3;
gmm_rrt_options.stop_criteria = 3;
gmm_rrt_options.parallel = false;
gmm_rrt_options.display = true;
options.gmm_rrt_options = gmm_rrt_options;
planner = plannerGMMRRT(ss,sv,options);
planner.init();

%% Test GMM model
