function neg_log= get_neg_log(x,pi,pds)
%GET_NEG_LOG Summary of this function goes here
%   Detailed explanation goes here
log_sum = 0;
for i=1:size(x,2)
    sum_sum = 0;
    for j = 1:size(pds,2)
        sum_sum = sum_sum + pi(j)*pdf(pds(j),x(i));
    end
    log_sum = log_sum + log(sum_sum);
end
neg_log = -log_sum;

