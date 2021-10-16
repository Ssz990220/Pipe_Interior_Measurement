function [new_gm,new_samples,new_idx] = merge_with_new_dist(old_gm,gm_added,old_samples, ...
    old_idx, add_samples, added_samples_idx, options)
%MERGE_WITH_NEW_DIST Merge existing gm with new gm distribution
%   Detailed explanation goes here
% new_idx = zeros(length(added_samples_idx),1);
n_added = gm_added.NumComponents;
% Collect Data
mu_added = gm_added.mu;
Sigma_added = gm_added.Sigma;
mu_old = old_gm.mu;
Sigma_old = old_gm.Sigma;
% Distance
bhat_dis = bhat_dis_new(mu_old, Sigma_old, mu_added, Sigma_added);
[min_dis, min_idx] = min(bhat_dis,[],1);
merge_flag = min_dis < options.bhat_dis_threshold;
new_mu = mu_old;
new_Sigma = Sigma_old;
list_idx = 1:n_added;
% Merge Section
idx_counter = size(old_idx,1) + 1;
for i = 1:old_gm.NumComponents
    if any(min_idx(merge_flag)==i)
        samples_i = old_samples(old_idx == i,:);
        for j = 1:sum(min_idx(merge_flag)==i)
            idxs = list_idx(min_idx(merge_flag)==i);
            idx = idxs(j);
            samples_i = [samples_i;add_samples(added_samples_idx==idx,:)];
            n_new_samples = sum(added_samples_idx==idx);
%             new_idx(idx_counter:idx_counter + n_new_samples) = ones(n_new_samples,1) * i;
            new_samples(idx_counter:idx_counter + n_new_samples,:) = ...
                add_samples(added_samples_idx==idx,:);
            idx_counter = idx_counter + n_new_samples;
        end
        gm = fitgmdist(samples_i,1);
        new_mu(i,:) = gm.mu;
        new_Sigma(:,:,i) = gm.Sigma;
    end
end
% Add distant gms
idxs = list_idx(~merge_flag);
for i = 1:size(idxs,2)
    idx = idxs(i);
    new_mu = [new_mu;gm_added.mu(idx,:)];
    new_Sigma = cat(3,new_Sigma,gm_added.Sigma(:,:,idx));
    n_new_samples = sum(added_samples_idx==idx);
    new_samples(idx_counter+1:idx_counter + n_new_samples, :) =...
        add_samples(added_samples_idx == idx,:);
%     new_idx(idx_counter+1:idx_counter + n_new_samples) = ones(n_new_samples,1)*(i+old_gm.NumComponents);
    idx_counter = idx_counter + n_new_samples;
end
new_gm = gmdistribution(new_mu, new_Sigma);
% Debug
new_idx = cluster(new_gm, cvt_2n_space(new_samples));
end