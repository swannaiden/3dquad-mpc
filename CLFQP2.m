%%
clear


% constants
e2 = 1;
eta2 = 25;
J = 1;
k2 = 1;
delta = .1;
g = 9.81;
l2= 100000;

P = 1/2*[k2, e2; e2, J];
Q = [0,1/2*k2;1/2*k2,e2];

x0 = [0,0,pi,0,0,0];


T = 15;
dt = .1;
N = T/dt;
x = zeros(6,N+1);
x(:,1) = x0;
%%
for i = 1:N

    xdes = [0,0,0,0,0,0]';
    
    % some PID controller here
    
    pitchE = [x(3,i)-xdes(3),x(6,i)-xdes(6)]';
    V(:,i) = pitchE'*P*pitchE;
    Vdot = @(U) pitchE'*Q*pitchE + U(2)/J*(J*pitchE(2)+e2*pitchE(1));
     
    
    C = diag([1,1,0]);
    cost = @(U) 1/2*U'*C*U+l2*U(3)^2;
    u0 = [0,0,0]';
    
    U(:,i) = fmincon(cost, u0, [], [], [], [], [], [], @(U) constrSimple(U,V(:,i),Vdot,delta, eta2));
    
    CLF_value(:,i) = double( pitchE'*Q*pitchE + eta2*V(:,i) -U(3,i))';%+ U(2,i)/J*(J*pitchE(2)+e2*pitchE(1));
    % euler integration 
    x(:,i+1) = qdynamics(x(:,i), [U(1,i),U(2,i)]'+[0,0]')*dt + x(:,i);

end

%% Plot shit


t = linspace(0,T,N+1);

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
subplot(1,3,1)
plot(t(1:end-1), U(1,:))
xlabel('t (s)')
ylabel('thrust')

subplot(1,3,2)
plot(t(1:end-1), U(2,:))
xlabel('t (s)')
ylabel('torque')

subplot(1,3,3)
plot(t(1:end-1), CLF_value)
xlabel('t (s)')
ylabel('\delta')

figure(3)
plot(t,x(3,:))
xlabel('t (s)')
ylabel('\phi (t)')

