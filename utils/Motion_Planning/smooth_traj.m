function q_target = smooth_traj(qs,target_u)
%B_SPLINE Fifth order B-Spline passes qs
[u,u_bar]=get_u4b_spline(qs);
% Build N Matrix
N_matrix = zeros(N+1,N+1);
for i = 0:N
    for j = 0:N
        N_matrix(j+1,i+1) = BaseFunction(i,5,u_bar(j+1),u);
    end
end
qu_s = zeros(size(qs));
for i = 1:size(qs,2)
    qu_s(:,i)=N_matrix\qs(:,i);
end
% Target Pose
N_target = zeros(size(target_u,2),N+1);
for i = 0:N
    for j = 1:length(target_u)
        N_target(j,i+1) = BaseFunction(i,5,target_u(j),u);
    end
end
q_target=N_target*qu_s;
end

