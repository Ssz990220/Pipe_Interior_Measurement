function options = GMM_RRT_Config(start, target)
%GMM_RRT_CONFIG The options for learning GMM model
options.num_init_sampler = 1000;
options.display_init_result = true;
options.randSampleProb = 0.5;
options.col_false_positive_prob = 0.03;
options.col_true_negative_prob = 0.8;
% Heuristic Sampling
options.add_trajectory_based_sample = true;
options.num_init_sampler_traj = 1000;
options.start = start;
options.target = target;
options.var = 0.2;
% Fixed GMM
options.fixed_gmm = false;
fixed_gmm_options.num_component = 7;
options.fixed_gmm_options = fixed_gmm_options;
% Collision GMM Model Parameters
gmm_rrt_col_options.max_iter = 10;
gmm_rrt_col_options.bhat_dis_threshold = 2;
gmm_rrt_col_options.start_merge_threshold = 3;
gmm_rrt_col_options.stop_criteria = 5;
gmm_rrt_col_options.parallel = false;
gmm_rrt_col_options.display = true;
options.gmm_rrt_col_options = gmm_rrt_col_options;
% Collision Free GMM Model Parameters
gmm_rrt_free_options.max_iter = 10;
gmm_rrt_free_options.bhat_dis_threshold = 2;
gmm_rrt_free_options.start_merge_threshold = 3;
gmm_rrt_free_options.stop_criteria = 5;
gmm_rrt_free_options.parallel = false;
gmm_rrt_free_options.display = true;
options.gmm_rrt_free_options = gmm_rrt_free_options;
end

