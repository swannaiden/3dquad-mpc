function xdot = qdynamics(x, u)
    
    %const
    m = 1;
    Ixx = 1;
    g = 9.81;
    
    xdot(1:3) = x(4:6);
    xdot(4) = -u(1)*sin(x(3))/m;
    xdot(5) = u(1)*cos(x(3))/m - g;
    xdot(6) = u(2)/Ixx;

    xdot = xdot';
end


