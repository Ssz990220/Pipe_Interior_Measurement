function bhat_dis =  bhat_dis_new(mu_old, Sigma_old, mu_added, Sigma_added)
num_old = size(mu_old,1);
num_new = size(mu_added,1);
bhat_dis = zeros(num_old, num_new);
for i = 1:num_old
    for j = 1:num_new
        bhat_dis(i,j) = 1/8*(mu_old(i,:)-mu_added(j,:))*inv((Sigma_old(:,:,i)...
            +Sigma_added(:,:,j))/2)*(mu_old(i,:)-mu_added(j,:))'+1/2*log(det((Sigma_old(:,:,i)...
            +Sigma_added(:,:,j))/2)/sqrt(det(Sigma_old(:,:,i)*Sigma_added(:,:,j))));
    end
end
end
