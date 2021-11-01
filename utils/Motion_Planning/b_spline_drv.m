function [p_u,v_u,a_u,NodeVector] = b_spline_drv(P,k,u)
n = size(P,2)-1;
[p_u,NodeVector4v,NodeVector] = b_spline(P,k,u);
% 曲线导矢计算
for i = 0:n-1
    Q(:,i+1) = k/(NodeVector4v(i+k+1)-NodeVector4v(i+1))*(P(:,i+2)-P(:,i+1));
end
[v_u,NodeVector4a] = BSpline(Q,k-1,u);

for i = 0:n-2
    Q2(:,i+1) = (k-1)/(NodeVector4a(i+k)-NodeVector4a(i+1))*(Q(:,i+2)-Q(:,i+1));
end
[a_u,~] = BSpline(Q2,k-2,u);

end