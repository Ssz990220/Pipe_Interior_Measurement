function [gm,new_states,new_idx] = update_gm_with_new_samples(states, old_gm, old_states, old_idx)
%UPDATE_GM_WITH_NEW_SAMPLES Summary of this function goes here
%   Detailed explanation goes here
num_states = size(states,1);
nearest_gm_idx = zeros(num_states,1);
for i = 1:num_states
   dis = mahal(old_gm, cvt_2n_space(states(i,:)));
   [~,nearest_gm_idx(i)] = min(dis);
end
mu = old_gm.mu;
Sigma = old_gm.Sigma;
for i = 1:old_gm.NumComponents
   state_i = states((nearest_gm_idx==i),:);
   if size(state_i,1)>0
       state_i = [state_i;old_states(old_idx==i,:)];
       state_i_2n = cvt_2n_space(state_i);
       dist_i = fitgmdist(state_i_2n,1);
       mu(i,:)=dist_i.mu;
       Sigma(:,:,i)=dist_i.Sigma;
   end
end
gm = gmdistribution(mu, Sigma);
new_states = [old_states;states];
new_idx = cluster(gm,cvt_2n_space(new_states));
end

