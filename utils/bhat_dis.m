function bhat_dis = bhat_dis(mu,Sigma,pair)
%BHAT_DIS Summary of this function goes here
%   Detailed explanation goes here
if nargin==3
    bhat_dis = zeros(1,size(pair,1));
    for i = 1:size(pair,1)
    bhat_dis(i) = 1/8*(mu(pair(i,1),:)-mu(pair(i,2),:))*inv((Sigma(:,:,pair(i,1))+Sigma(:,:,pair(i,2)))/2)*(mu(pair(i,1),:)-mu(pair(i,2),:))'...
        +1/2*log(det((Sigma(:,:,pair(i,1))+Sigma(:,:,pair(i,2)))/2)/sqrt(det(Sigma(:,:,pair(i,1))*Sigma(:,:,pair(i,2)))));
    end
else
    n_cluster = length(mu);
    bhat_dis = ones(n_cluster, n_cluster)*inf;
    counter = 1;
    for i = 1:n_cluster
        for j = (i+1):n_cluster
            bhat_dis(i,j) = 1/8*(mu(i,:)-mu(j,:))*inv((Sigma(:,:,i)+Sigma(:,:,j))/2)*(mu(i,:)-mu(j,:))'...
        +1/2*log(det((Sigma(:,:,i)+Sigma(:,:,j))/2)/sqrt(det(Sigma(:,:,i)*Sigma(:,:,j))));
        counter = counter + 1;
        end
    end
end
end
