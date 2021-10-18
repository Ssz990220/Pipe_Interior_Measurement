clc;
clear;
close all
figure
axis equal
mu = [5,5;6,3;6,6;10,2];
sigma(:,:,1) = [1,-0.6;-0.6,1];
sigma(:,:,2) = [1,0;0,1];
sigma(:,:,3) = [4,-2;-2,4];
sigma(:,:,4) = [1,-0.7;-0.7,1];
n_cluster = size(mu,1);
proportion = rand(size(mu,1),1);
proportion = proportion/sum(proportion);
gm = gmdistribution(mu,sigma,proportion);
X = random(gm,1000);
plot(X(:,1),X(:,2),'.','MarkerSize',15);


%% EM
gmBest = fitgmdist(X,n_cluster,'CovarianceType','full','SharedCovariance',false);
plot_gmm_dist(gmBest,X);
%% Incremental EM
options.max_iter = 100;
options.bhat_dis_threshold = 0.3;
options.start_merge_threshold = 1;
options.stop_criteria = 5;
options.parallel = false;
options.display = true;
gmm_k_means = GMM_K_means(X,10,options);
gmm_k_means.BUILD_GMM();

for i = 1:size(gmm_k_means.gmm_models,2)
    f = plot_gmm_dist(gmm_k_means.gmm_models{i},X,false);
    saveas(f,sprintf('%d.png',i));
end