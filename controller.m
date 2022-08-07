function u = controller(x,v)
    %const
    m = 1;
    g = 9.81;
    
    
    k_hover = [10,5]; % 10, 5
    k_angle = [65,25]; % 65, 15
    k_d_vel = .35;
    
    %implement velocity control in x
    des_angle = min(max(-k_d_vel*(x(4)-v), -pi/6),pi/6);

    u(1) = m*g - k_hover*x([2,5]);
    u(2) =-k_angle*(x([3,6])+[des_angle; 0]);

end