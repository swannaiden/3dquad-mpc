clear

x0 = [0,0];
circPos = [4.01,4]';
b = 3;
T = 8;
dt = .01;
N = T/dt;
x = zeros(2,N+1);
x(:,1) = x0;

eta = 1;
delta = 0;
x_des = [10,10]';
for i = 1:N
    
       
    h(:,i) = (x(1,i)-circPos(1))^2+(x(2,i)-circPos(2))^2-b^2;
    hdot = @(U) 2*(x(1,i)-circPos(1))*U(1)+2*(x(2,i)-circPos(2))*U(2);
    
    u_des = x_des - x(:,i);
    cost = @(U) (U(1)-u_des(1))^2+(U(2)-u_des(2))^2;
    
    u0 = [0,0]';
    
    U1(:,i) = fmincon(cost, u0, [], [], [], [], [], [], @(U) constrCBF(U,h(:,i),hdot,eta, delta))
    constr(:,i) = double(-hdot(U1(:,i)) - eta*h(:,i) +delta);
    

    x(:,i+1) = x(:,i)+U1(:,i)*dt;

end


%%

t = linspace(0,T,N+1);

figure(1)
subplot(2,1,1)
plot(t,x(1,:))
xlabel('t (s)')
ylabel('x(t)')

subplot(2,1,2)
plot(t,x(2,:))
xlabel('t (s)')
ylabel('y(t)')


%%

%% Plot Trajectory
b = 3;
circPos = [4,4];
figure(6)
hold on
plot(x(1,:), x(2,:),'b')
hold on
x1 = linspace(0,2*pi);
% plot(b*cos(x1)+circPos(1), b*sin(x1)+circPos(2))
% plot(10,10, 'go','MarkerSize',20, 'LineWidth',2)
axis([-2 12 0 12])
xlabel('x (m)')
ylabel('y (m)')
title('Avoiding Obstacle')
for i = 1:N
   if(mod(i, 20)==0)
%        plot(x(1,i), x(2,i), 'r*') 
%        x3 = [0,1]';
%        theta = x(3,i);
%        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%        x3 = R*x3;
%        quiver(x(1,i), x(2,i), x3(1), x3(2),'LineWidth',1);
   end
end
axis equal
legend('backup barrier', 'obstacle', 'Goal', 'MPC', 'desired controller','CBF')

%%

%% Animate Trajectory

%% animate trajectory
% record video
myVideo = VideoWriter('MPC'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
b = 3;
circPos = [4,4];
figure(4)
hold on
% plot(x(1,:), x(2,:))
hold on
x1 = linspace(0,2*pi);
plot(b*cos(x1)+circPos(1), b*sin(x1)+circPos(2), 'b')
plot(10,10, 'go','MarkerSize',20, 'LineWidth',2)
axis equal
axis([-2 12 0 12])
set(gcf,'color','w');

for i = 1:N
   if(mod(i, 5)==0)
%        plot(x(1,1:i), x(2,1:i),'r')

%        plot(x(1,i), x(2,i), 'r*')
       if(mod(i,10)==0)
        
       
%        x3 = [0,1]';
%        theta = x(3,i);
%        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%        x3 = R*x3;
%        quiver(x(1,i), x(2,i), x3(1), x3(2),.5,'r','LineWidth',1);
        end

    xlabel('x (m)')
   ylabel('y (m)')
   title('MPC')
   pause(.05)
   frame = getframe(gcf); %get frame
   writeVideo(myVideo, frame);
   end
   
end
axis equal
close(myVideo)

