function [uout, lambda, h] = barrier(x, uin)

    % first calculate h_min
    
    % integrate along flow of the backup controller w/ piccard iteration
    
    barrier_pos = 2;
    circPos = [4,4];
    circRad = 3;
    dt = .05;
    time = 2.5;
    %x(:,1) = x0;
    t = 0:dt:time-dt;
    h = 1000000;
    
    for i = 2:(time/dt)
        u = controller(x,0);
        x = x+qdynamics(x, u)*dt;
        % calc distance from barrier wall at barrier_pos
%         barrier_dis = barrier_pos - sqrt(x(1)^2);
        %circl barrier
        barrier_dis = sqrt((x(1)-circPos(1))^2+(x(2)-circPos(2))^2-circRad^2);
        if(h > barrier_dis)
            h = barrier_dis;
        end
    end
    
    if(h < 0)
        h = 0;
    end
    % smooth regulation function
    lambda = 1-exp(-.5*h^(1/3));

    uout = uin*(lambda)+controller(x,-1*.15)'*(1-lambda); %.15


end