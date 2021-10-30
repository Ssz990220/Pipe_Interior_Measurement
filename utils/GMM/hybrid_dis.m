function hybrid_dis = hybrid_dis(state, GMM_col_model, GMM_free_model)
   n = size(state,1);
   state_2n = cvt_2n_space(state);
   if n > 1
       hybrid_dis = zeros(1,n);
       for i = 1:n
            hybrid_dis(i) = hybrid_one_dis(state_2n(i,:),GMM_col_model, GMM_free_model);
       end
   else
      hybrid_dis = hybrid_one_dis(state_2n,GMM_col_model, GMM_free_model); 
   end
end

function hybrid_dis = hybrid_one_dis(state_2n, GMM_col_model, GMM_free_model)
    dis_col = min(mahal(GMM_col_model,state_2n));
    dis_free = min(mahal(GMM_free_model,state_2n));
    hybrid_dis = dis_col - dis_free;
end