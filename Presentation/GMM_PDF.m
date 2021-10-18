clear;
clc;
close all;
mu1 = [1 2];          % Mean of the 1st component
sigma1 = [2 0; 0 .5]; % Covariance of the 1st component
mu2 = [-3 -5];        % Mean of the 2nd component
sigma2 = [1 0; 0 1];  % Covariance of the 2nd component
mu3 = [4,2];
sigma3 = [2,0.6;0.6,4];
mu4 = [3.5,3];
sigma4 = [2,0.4;0.4,2];
rng('default') % For reproducibility
r1 = mvnrnd(mu1,sigma1,1000);
r2 = mvnrnd(mu2,sigma2,1000);
r3 = mvnrnd(mu3,sigma3,1000);
r4 = mvnrnd(mu4,sigma4,1000);
X = [r1; r2; r3; r4];


gm = fitgmdist(X,4);

scatter(X(:,1),X(:,2),10,'.') % Scatter plot with points of size 10
hold on
gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(gm,[x0 y0]),x,y);
fcontour(gmPDF,[-8 6])