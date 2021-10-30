function C = coriolis_m(robot, q, qd)

    % we need to create a clone robot with no friction, since friction
    % is also proportional to joint velocity
    robot2 = robot;
    robot2.Gravity = [0,0,-9.81];

    N = size(q,2);
    C = zeros(N,N);
    Csq = zeros(N,N);


    % find the torques that depend on a single finite joint speed,
    % these are due to the squared (centripetal) terms
    %
    %  set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
    for j=1:N
        QD = zeros(1,N);
        QD(j) = 1;
        tau = inverseDynamics(robot2, q, QD, zeros(size(q)));
        Csq(:,j) = Csq(:,j) + tau.';
    end

    % find the torques that depend on a pair of finite joint speeds,
    % these are due to the product (Coridolis) terms
    %  set QD = [1 1 0 ...] then resulting torque is due to 
    %    qd_1 qd_2 + qd_1^2 + qd_2^2
    for j=1:N
        for k=j+1:N
            % find a product term  qd_j * qd_k
            QD = zeros(1,N);
            QD(j) = 1;
            QD(k) = 1;
            tau = inverseDynamics(robot2, q, QD, zeros(size(q)));
            C(:,k) = C(:,k) + (tau.' - Csq(:,k) - Csq(:,j)) * qd(j)/2;
            C(:,j) = C(:,j) + (tau.' - Csq(:,k) - Csq(:,j)) * qd(k)/2;
        end
    end

    C = C + Csq * diag(qd);
end
