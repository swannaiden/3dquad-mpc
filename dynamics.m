function dx = dynamics(x,u)

   %dx = zeros(10,1);
    dx = opti.variable(10,1);
    m = 1;
    g = -9.81;
    wx = u(2); wy = u(3); wz = u(4);
    dx(1:3) = x(8:10);
    dx(4:7) = 1/2*[0 -wx -wy -wz; wx 0 -wz -wy; wy -wz 0 wx; wz -wy -wx 0]*x(4:7);
    dx(8:10) = quatrotate(x(4:7)', [0,0,u(1)])/m;
    dx(10) = dx(10)+g;
    
    dx = dx';
    

end