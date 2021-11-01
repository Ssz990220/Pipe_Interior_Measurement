function [u,u_bar] = get_u4b_spline(qs)
%GET_U4B_SPLINE Summary of this function goes here
%   Detailed explanation goes here
N = size(qs,1)-1;
delta_q = sqrt(sum((qs(2:end,:)-qs(1:end-1,:)).^2,2));
d = sum(delta_q);
delta = delta_q/d;
u_bar = cumsum(delta);
u_bar = u_bar(1:end-1);
u_bar = [0;u_bar;1];
u = zeros(1,N+7);
u(end-5:end) = 1;
for j = 1:N-5
    u(j+6)=1/5*sum(u_bar(j+1:j+5));
end
end

