clc; clear

addpath('C:\Users\swann\Documents\MATLAB\toolbox\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

% Car race along a track
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.

N = 100; % number of control intervals
% T = 10;

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(6,N+1); % state trajectory
posx   = X(1,:);
posy = X(2,:);
velx = X(4,:);
vely = X(5,:);
pos = X(1:2,:);
U = opti.variable(2,N);   % control trajectory (throttle)
T = opti.variable();      % final time

% ---- objective          ---------
opti.minimize(T); % race in minimal time

% ---- dynamic constraints --------

% f1 = [xdot; ydot; thetadot; 0; -g1; 0];
% g1 = [0,0;0,0;0,0;-sin(theta)/m, 0;cos(theta)/m,0;0,1/ixx];
ixx = 1;
m = 1;
grav = 9.81;
f = @(x,u) [x(4);x(5);x(6);0;-grav;0] + ...
    [0,0;0,0;0,0;-sin(x(3))/m, 0;cos(x(3))/m,0;0,ixx]*u; %[x(2);u-x(2)]; % dx/dt = f(x,u)

% dt = T/N; % length of a control interval
dt = T/N;
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% % ---- path constraints -----------
% limit = @(pos) 1-sin(2*pi*pos)/2;
% opti.subject_to(speed<=limit(pos)); % track speed limit
% opti.subject_to(0<=U<=1);           % control is limited
circPos = [4, 4];
circlRad = 3;
obstacle = @(pos) sqrt((pos(1,:)-circPos(1)).^2+(pos(2,:)-circPos(2)).^2);
opti.subject_to(obstacle(pos)>=circlRad);
opti.subject_to(-5<=U(2,:)<=5);           % control is limited
opti.subject_to(0<=U(1,:)<=20);           % control is limited



% ---- boundary conditions --------
% opti.subject_to(posx(1)==0);   % start at position 0 ...
% opti.subject_to(posy(1)==0);   % start at position 0 ...
% opti.subject_to(velx(1)==0); % ... from stand-still 
% opti.subject_to(vely(1)==0); % ... from stand-still 
opti.subject_to(X(1:6,1)==zeros(6,1));
opti.subject_to(X(4:5,N+1) == zeros(2,1));
opti.subject_to(posx(N+1)==10); % finish line at position 1
opti.subject_to(posy(N+1)==10);

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
opti.set_initial(velx, 1);
opti.set_initial(vely, 1);
opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

% ---- post-processing        ------

%%
t = linspace(0,sol.value(T),N+1);
x = sol.value(X);
u = sol.value(U);



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
plot(t(1:end-1), u(1,:))
xlabel('t (s)')
ylabel('thrust')

subplot(1,2,2)
plot(t(1:end-1), u(2,:))
xlabel('t (s)')
ylabel('torque')

%% Old plotting stuff

% figure
% hold on
% plot(t,sol.value(speed));
% plot(t,sol.value(pos));
% plot(t,limit(sol.value(pos)),'r--');
% stairs(t(1:end-1),sol.value(U),'k');
% xlabel('Time [s]');
% legend('speed','pos','speed limit','throttle','Location','northwest')
% print('OCP_sol','-dpng')
% 
% figure
% spy(sol.value(jacobian(opti.g,opti.x)))
% xlabel('decision variables')
% ylabel('constraints')
% print('jac_sp','-dpng')


%% Plot Trajectory
figure(3)
plot(x(1,:), x(2,:))
hold on
x1 = linspace(0,2*pi);
plot(circlRad*cos(x1)+circPos(1), circlRad*sin(x1)+circPos(2))
for i = 1:N
   if(mod(i, 20)==0)
       plot(x(1,i), x(2,i), 'r*') 
       x3 = [0,1]';
       theta = x(3,i);
       R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
       x3 = R*x3;
       quiver(x(1,i), x(2,i), x3(1), x3(2),'LineWidth',1);
   end
end
axis equal

%%
%% animate trajectory
% figure(2)
% for i = 1:length(x)
%     plot(x(1,1:i), x(2,1:i))
%     hold on
%     plot(10*ones(1,20),linspace(-1,1,20))
%     hold off
%     theta = 5*x(3,i);
%     R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
%     x1 = R*[0;1];
% %     y1 = R*[1;0];
%     hold on
%     %quiver([x(1,i), x(1,i)], [x(2,i),x(2,i)], [x1(1), x1(2)],[x1(2), x1(1)])
%     quiver([x(1,i),x(1,i)], [x(2,i), x(2,i)], [x1(2), -x1(2)],[-x1(1), x1(1)], .15)
%     hold off
%     %quiver(x(1,i), x(2,i), -x1(1),-x1(2))
%     %axis([5 11 -1 1])
%     drawnow
% end

% figure(3)
% z = zeros(size(x(1,:)));
% surface([x(1,:);x(1,:)],[x(2,:);x(2,:)],[z;z],[t;t],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',2);
% colorbar

