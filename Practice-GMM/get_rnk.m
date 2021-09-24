function rnk = get_rnk(x,pi,pds)
%GET_RNK Summary of this function goes here
%   Detailed explanation goes here
rnk = zeros(size(x,2),size(pds,2));
N_dist = zeros(size(rnk));
for i=1:size(pds,2)
    N_dist(:,i)=pi(i)*pdf(pds(i),x);
end
denom = sum(N_dist,2);
for i=1:size(pds,2)
    rnk(:,i)=N_dist(:,i)./denom;
end
end

