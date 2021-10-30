function [ds, cs, gs] = get_dcg(robot,qs,qds,qdds)
%GET_DCG Summary of this function goes here
%   Detailed explanation goes here
    len = size(qs,1);
    n_dof = size(qs,2);
    Ds = zeros(n_dof, n_dof, len);
    Cs = zeros(n_dof, n_dof, len);
    gs = zeros(len,n_dof);
    parfor i = 1:len
        Ds(:,:,i) = massMatrix(robot, qs(i,:));
        Cs(:,:,i) = coriolis_m(robot, qs(i,:),qds(i,:));
        gs = gravityTorque(robot, qs(i,:));
    end
    qds = reshape(qds', [n_dof, 1, size(qds,1)]);
    qdds = reshape(qdds', [n_dof, 1, size(qdds,1)]);
    ds = pagemtimes(Ds, qds);
    ds = reshape(ds, [n_dof, size(ds,3)])';
    cs = pagemtimes(Ds, qdds) + pagemtimes(Cs, qds);
    cs = reshape(cs, [n_dof, size(cs,3)])';           
end