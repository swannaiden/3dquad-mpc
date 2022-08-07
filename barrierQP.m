%% Dynamics
clear

% mxdd = fv1
% mydd = fv2

%constants
g1 = 9.81;
m = 1;
ixx = 1;
k1 = 1;
epsilon = 1;
eta = 1;
% syms x y theta xdot ydot thetadot real
% vars = [x, y, theta, xdot, ydot, thetadot];
% 
% f = [xdot; ydot; thetadot; 0; -g; 0];
% g = [0,0;0,0;0,0;-sin(theta)/m, 0;cos(theta)/m,0;0,1/ixx];
% 
% alpha = @(r) 1*r; % Class K function

syms x y xdot ydot F f1 f2 real
vars = [x, y, xdot, ydot]';

f = [xdot; ydot; 0; 0];
g = [0,0;0,0;1,0;0,1];

ex = [x;y];
ev = [xdot;ydot];

V = 1/2*m*dot(ev,ev)+1/2*k1*dot(ex,ex)+epsilon*dot(ex,ev);
Lfh = jacobian(V,vars)*f;
Lgh = jacobian(V,vars)*g;
% F = [f1; f2];

x0 = [-1,-1,0,0]';
Lfh = subs(Lfh, vars, x0);
Lgh = subs(Lgh, vars, x0);
V = subs(V, vars, x0);

% Vdot = @(U) Lfh+Lgh*[U(1); U(2)];
% constr1 = @(U) Lfh+Lgh*[U(1); U(2)] + eta*V;
%c = @(x) (x(1)-1/3)^2 + (x(2)-1/3)^2 - (1/3)^2;

Q = eye(2);
cost = @(U) 1/2*U'*Q*U;
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
ceq = [];
u0 = [0,0]';
%st Vdot + eta*V <= 0
% nonlcon = @circlecon;
% nonlcon = @(x) deal(c,[]);
nonlcon = @constr1;

U = fmincon(cost, u0, A, b, Aeq, beq, lb, ub, @(U) nonlcon_pquadthrustvec(U,Lfh,Lgh,V,eta))

%%
% now do seccond optimization
Fc = max(dot(U, [cos(x0(3)), sin(x0(3))]'), 0);
thetac = atan(U(2)/U(1));
thetacdot = 0;
k2 = 1;
e2 = 1;
x0 = [0,0,0,0,0,0];
%%
% Fc = 1;
% thetac = 0.001;

clearvars f g V
syms x y theta xdot ydot thetadot real
vars = [x, y, theta, xdot, ydot, thetadot];

f = [xdot; ydot; thetadot; 0; -g1; 0];
g = [0,0;0,0;0,0;-sin(theta)/m, 0;cos(theta)/m,0;0,1/ixx];

V = 1/2*ixx*(thetadot-thetacdot)^2+1/2*k2*(theta-thetac)^2+e2*(theta - thetac)*(thetadot-thetacdot);

Lfh = jacobian(V,vars)*f;
Lgh = jacobian(V,vars)*g;
% F = [f1; f2];

Lfh = subs(Lfh, vars, x0);
Lgh = subs(Lgh, vars, x0);
V = subs(V, vars, x0);

delta = 0;
L1 = 1;
L2 = 1;
Q = eye(2);
eta2 = 1;
cost = @(U) L1*1/2*(U(1)- Fc)^2+ 1/2*U(2)^2+1/2*L2*delta^2;
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
ceq = [];
u0 = [0,0]';

% run fmincon

U = fmincon(cost, u0, A, b, Aeq, beq, lb, ub, @(U) constr2(U,Lfh,Lgh,V,eta2,delta))

ceq = double( Lfh+Lgh*[U(1); U(2)] + eta2*V -delta)

% function [c,ceq] = nonlcon(x)
%     c(1) = (x(1)^2)/9 + (x(2)^2)/4 - 1;
%     c(2) = x(1)^2 - x(2) - 1;
%     ceq = [];
% end
%% Circle Example
% clear
% syms x
% fun = @(x)100*(x(2)-x(1)^2)^2 + (1-x(1))^2;  
% lb = [0,0.2];
% ub = [0.5,0.8];  
% A = [];
% b = [];
% Aeq = [];
% beq = [];  
% 
% x0 = [1/4,1/4];  
% 
% nonlcon = @circlecon;
% x2 = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)   
% 
% 
% % function [c,ceq] = circlecon(x)
% %     c = (x(1)-1/3)^2 + (x(2)-1/3)^2 - (1/3)^2;
% %     ceq = [];
% % end
