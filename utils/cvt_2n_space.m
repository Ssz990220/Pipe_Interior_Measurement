function states_2n = cvt_2n_space(states)
    states_2n = zeros(size(states,1),size(states,2)*2);
    for i = 1:size(states,2)
        states_2n(:,(i-1)*2+1:i*2)=[cos(states(:,i)),sin(states(:,i))];
    end
end

