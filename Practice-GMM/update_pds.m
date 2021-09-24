function pds = update_pds(mu,sigma,pds)
%UPDATE_PDS Summary of this function goes here
%   Detailed explanation goes here
for i = 1:size(pds,2)
   pds(i).mu=mu(i);
   pds(i).sigma = sigma(i);
end
end

