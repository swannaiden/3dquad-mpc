%% Dynamics
clear

% mxdd = fv1
% mydd = fv2


x0A = [0,-1,0,0,0,0]';
T = 10;
dt = .1;
N = T/dt;
xState = zeros(6,N+1);
xState(:,1) = x0A;
i = 1;
%%
for i = 1:N
%%s
if(mod(N, 50))
    n
end

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

x0 = [xState(1,i), xState(2,i), xState(4,i), xState(5,i)]';
% x0 = [-1,-1,0,0]';
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

U = fmincon(cost, u0, A, b, Aeq, beq, lb, ub, @(U) nonlcon_pquadthrustvec(U,Lfh,Lgh,V,eta));
U5(:,i) =U;
%%
% now do seccond optimization
Fc = max(dot(U, [sin(x0(3)),cos(x0(3))]'), -100);
thetac = atan(U(1)/U(2));
thetacdot = 0;
k2 = 1;
e2 = 1;
x0 = xState(:,i);
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

Lfh = subs(Lfh, vars', x0);
Lgh = subs(Lgh, vars', x0);
V = subs(V, vars', x0);

delta = 0.1;
L1 = 1;
L2 = 1000000;
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

U = fmincon(cost, u0, A, b, Aeq, beq, lb, ub, @(U) constr2(U,Lfh,Lgh,V,eta2,delta));
UF(:,i) = U;
ceq = double( Lfh+Lgh*[U(1); U(2)] + eta2*V -delta);

xState(:,i+1) = qdynamics(xState(:,i), U+[g1,0]')*dt + xState(:,i);

end


%%


t = linspace(0,T,N+1);
x = xState;
u = UF;



figure(1)
subplot(2,3,1)
plot(t,x(1,:))
xlabel('t (s)')
ylabel('x(t)')

subplot(2,3,2)
plot(t,x(2,:))
xlabel('t (s)')
ylabel('y(t)')

subplot(2,3,3)
plot(t,x(3,:))
xlabel('t (s)')
ylabel('\phi (t)')

subplot(2,3,4)
plot(t,x(4,:))
xlabel('t (s)')
ylabel('dx(t)')

subplot(2,3,5)
plot(t,x(5,:))
xlabel('t (s)')
ylabel('dy(t)')

subplot(2,3,6)
plot(t,x(6,:))
xlabel('t (s)')
ylabel('d\phi (t)')

figure(2)
subplot(1,2,1)
plot(t(1:end-1), UF(1,:))
xlabel('t (s)')
ylabel('thrust')

subplot(1,2,2)
plot(t(1:end-1), UF(2,:))
xlabel('t (s)')
ylabel('torque')

figure(3)
subplot(1,2,1)
plot(t(1:end-1), U5(1,:))
xlabel('t (s)')
ylabel('x thrust')

subplot(1,2,2)
plot(t(1:end-1), U5(2,:))
xlabel('t (s)')
ylabel('y thrust')