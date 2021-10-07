clc
clear
close all
Dataset = [-3,-2,5,-1,0,2,4,5];
pi = [1/3,1/3,1/3];
mu = [-4,0,8];
sigma = [1,0.2,3];
pd1 = makedist('Normal','mu',mu(1),'sigma',sigma(1));
pd2 = makedist('Normal','mu',mu(2),'sigma',sigma(2));
pd3 = makedist('Normal','mu',mu(3),'sigma',sigma(3));
pds = [pd1,pd2,pd3];

while 1
   rnk = get_rnk(Dataset,pi,pds);
   Nk_all = sum(rnk,1);
   mu_new = zeros(size(mu));
   sigma_new = zeros(size(sigma));
   for i = 1:size(pds,2)
      mu_new(i)=sum(rnk(:,i).*Dataset')/Nk_all(i);
      sigma_new(i)=sum(rnk(:,i).*((Dataset-mu(i)).^2)')/Nk_all(i);
   end
   pi = Nk_all/size(Dataset,2);
   mu = mu_new;
   sigma = sigma_new;
   pds = update_pds(mu,sigma,pds);
   neg_log = get_neg_log(Dataset,pi,pds)
end