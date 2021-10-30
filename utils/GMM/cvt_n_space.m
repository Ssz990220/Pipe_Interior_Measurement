function states = cvt_n_space(states_2n)
    dim = size(states_2n, 2)/2;
    length = size(states_2n,1);
    states = zeros(length,dim);
    for i = 1:dim
        states(:,i)=atan2(states_2n(:,2*i),states_2n(:,2*i-1));
    end
end

