clc;
clear;
mu = (rand(10,2)-0.5)*20;
sigma = [1 1]; % shared diagonal covariance matrix
gm = gmdistribution(mu,sigma);
X = random(gm, 2000);
[n,p] = size(X);
rng(1) % For reproducibility

% figure
% plot(X(:,1),X(:,2),'.','MarkerSize',15)
% title('Fisher''s Iris Data Set')
% xlabel('Petal length (cm)')
% ylabel('Petal width (cm)')
gmBest = fitgmdist(X,5,'CovarianceType','full','SharedCovariance',false);
clusterX = cluster(gm,X);
kGMM = gmBest.NumComponents;
d = 500;
x1 = linspace(min(X(:,1)) - 2,max(X(:,1)) + 2,d);
x2 = linspace(min(X(:,2)) - 2,max(X(:,2)) + 2,d);
[x1grid,x2grid] = meshgrid(x1,x2);
X0 = [x1grid(:) x2grid(:)];
mahalDist = mahal(gmBest,X0);
threshold = sqrt(chi2inv(0.99,2));

figure
h1 = gscatter(X(:,1),X(:,2),clusterX);
hold on
for j = 1:kGMM
    idx = mahalDist(:,j)<=threshold;
    Color = h1(j).Color*0.75 + -0.5*(h1(j).Color - 1);
    h2 = plot(X0(idx,1),X0(idx,2),'.','Color',Color,'MarkerSize',1);
    uistack(h2,'bottom')
end
plot(gmBest.mu(:,1),gmBest.mu(:,2),'kx','LineWidth',2,'MarkerSize',10)
title('Clustered Data and Component Structures')
xlabel('Petal length (cm)')
ylabel('Petal width (cm)')
% legend(h1,'Cluster 1','Cluster 2','Cluster 3','Location','NorthWest')
hold off
%%
options.max_iter = 100;
options.bhat_dis_threshold = 0.5;
options.start_merge_threshold = 1;
options.stop_criteria = 3;
options.parallel = false;
options.display = true;
gmm_k_means = GMM_K_means(X,10,options);
gmm_k_means.BUILD_GMM();
gmBest = gmm_k_means.gmm_final;


clusterX = cluster(gmBest,X);
kGMM = gmBest.NumComponents;
d = 500;
x1 = linspace(min(X(:,1)) - 2,max(X(:,1)) + 2,d);
x2 = linspace(min(X(:,2)) - 2,max(X(:,2)) + 2,d);
[x1grid,x2grid] = meshgrid(x1,x2);
X0 = [x1grid(:) x2grid(:)];
mahalDist = mahal(gmBest,X0);
threshold = sqrt(chi2inv(0.99,2));

figure
h1 = gscatter(X(:,1),X(:,2),clusterX);
hold on
for j = 1:kGMM
    idx = mahalDist(:,j)<=threshold;
    Color = h1(j).Color*0.75 + -0.5*(h1(j).Color - 1);
    h2 = plot(X0(idx,1),X0(idx,2),'.','Color',Color,'MarkerSize',1);
    uistack(h2,'bottom')
end
plot(gmBest.mu(:,1),gmBest.mu(:,2),'kx','LineWidth',2,'MarkerSize',10)
title('Clustered Data and Component Structures')
xlabel('Petal length (cm)')
ylabel('Petal width (cm)')
% legend(h1,'Cluster 1','Cluster 2','Cluster 3','Location','NorthWest')
hold off