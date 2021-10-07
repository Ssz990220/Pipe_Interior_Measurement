clc;
clear;
mu = (rand(5,2)-0.5)*100;
sigma = [1 1]; % shared diagonal covariance matrix
gm = gmdistribution(mu,sigma);
X = random(gm, 1000);

%   Prepare distance matrix to accelerate further calculation
time_elp = zeros(2,5);
base = 2;
for p = 1+base:5+base
    n_sample = 2^p*10^2;
    data = random(gm,n_sample);
    n_data = size(data,1);
    dis_mat = zeros(n_data,n_data);
    dim_data = size(data,2);
    tic;
    for i = 1:n_data
       dis_mat(i,:)=sum((data(i,:)-data).^2,2);         % How to accelerate?
    end
    time_elp(1,p-base) = toc;
    fprintf('Iter %d took %.4f to complete',[p-base,time_elp(1,p-base)]);
end

for p = 1+base:5+base
    n_sample = 2^p*10^2;
    data = random(gm,n_sample);
    dim_data = size(data,2);
    n_data = size(data,1);
    tic;
    data_cube = repmat(data,1,1,n_data);
    data_cube2 = permute(data_cube,[3,2,1]);
    dis_mat = sum((data_cube - data_cube2).^2,2);
    dis_mat = permute(dis_mat,[1,3,2]);
    time_elp(2,p-base) = toc;
    fprintf('Iter %d took %.4f to complete',[p-base,time_elp(1,p-base)]);
end
figure
plot(1:5,time_elp(1,:))
hold on
plot(1:5,time_elp(2,:))
legend('for-loop','vectorized')
