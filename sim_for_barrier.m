%% simulate quadrotor using piccard iteration
clear

x0 = [0,0,0,0,0,0]';
m = 1;
g = 9.81;
dt = .01;
time = 10;
x(:,1) = x0;
x2(:,1) = x0;
t = 0:dt:time-dt;
x_des = [10,10]';
N = time/dt;
for i = 2:(time/dt)
    


    u(:,i) = [0,0]';
    v_des = .25*(x_des - x(1:2,i-1));
%     u(:,i) = .01*(x_des - x(1:2,i-1))+[0,9.81]';
%      u(:,i) = [0,0]';
    v = v_des(1);
    k_hover = [1,5]; % 10, 5
    k_angle = [65,25]; % 65, 15
    k_d_vel = .35;
    
    %implement velocity control in x
    des_angle = min(max(-k_d_vel*(x(4,i-1)-v), -pi/6),pi/6);

    u(1,i) = m*g - k_hover*(x([2,5],i-1)-[10,0]');
    u(2,i) =-k_angle*(x([3,6],i-1)+[des_angle; 0]);
%     u2 = controller(x2(:,i-1), cos(t(i)*2*3.14/10)*.6);
    [ubar, lambda(i),h(i)] = barrier1(x(:,i-1),u(:,i));
    ubar = u(:,i);
    x(:,i) = x(:,i-1)+qdynamics(x(:,i-1), ubar)*dt;
%     usent(:,i) = ubar;
%     x2(:,i) = x2(:,i-1)+qdynamics(x2(:,i-1), u2)*dt;

end


% figure(1)
% subplot(2,2,2)
% plot(t,x(1,:))
% hold on
% plot(t, 2*ones(1, length(t)), 'LineWidth',3, 'Color',[0,1,0])
% plot(t,x2(1,:))
% plot(t, -2*ones(1, length(t)), 'LineWidth',3, 'Color',[0,1,0])
% plot(t,x2(1,:))
% hold off
% xlabel('t (s)')
% ylabel('x(t)')
% axis([0 100 -5 5])
% % % legend('w/ Barrier', 'Barrier', 'w/o Barrier')
% title('Desired velocity with period \pi/10')
% 
% subplot(2,2,4)
% plot(t,lambda)
% xlabel('t (s)')
% ylabel('\lambda (i)')

% subplot(2,2,3)
% plot(t,x(3,:))
% xlabel('t (s)')
% ylabel('\phi (t)')

% subplot(2,2,2)
% plot(t,x(4,:))
% hold on
% plot(t, x2(4,:))
% xlabel('t (s)')
% ylabel('v_x(t)')
% % 
% subplot(2,2,4)
% plot(t,h)
% xlabel('t (s)')
% ylabel('h(i)')
% 
% subplot(2,3,6)
% plot(t,x(6,:))
% xlabel('t (s)')
% ylabel('d\phi (t)')
figure(1)
subplot(2,2,1)
plot(t,x(1,:))
xlabel('t (s)')
ylabel('x (t)')
subplot(2,2,2)
plot(t,x(2,:))
xlabel('t (s)')
ylabel('y (t)')
subplot(2,2,3)
plot(t,x(4,:))
xlabel('t (s)')
ylabel('xd (t)')
subplot(2,2,4)
plot(t,x(5,:))
xlabel('t (s)')
ylabel('yd (t)')


%%

%% Plot Trajectory

circPos = [4.01,4]';
b = 3;

figure(3)
hold on
plot(x(1,:), x(2,:))
hold on
x1 = linspace(0,2*pi);
plot(b*cos(x1)+circPos(1), b*sin(x1)+circPos(2))
for i = 1:N
   if(mod(i, 20)==0)
       plot(x(1,i), x(2,i), 'r*') 
%        x3 = [0,1]';
%        theta = x(3,i);
%        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%        x3 = R*x3;
%        quiver(x(1,i), x(2,i), x3(1), x3(2),'LineWidth',1);
   end
end
axis equal





%% animate trajectory
figure(2)
for i = 1:length(x)
    plot(x(1,1:i), x(2,1:i))
    hold on
    plot(10*ones(1,20),linspace(-1,1,20))
    hold off
    theta = 5*x(3,i);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    x1 = R*[0;1];
%     y1 = R*[1;0];
    hold on
    %quiver([x(1,i), x(1,i)], [x(2,i),x(2,i)], [x1(1), x1(2)],[x1(2), x1(1)])
    quiver([x(1,i),x(1,i)], [x(2,i), x(2,i)], [x1(2), -x1(2)],[-x1(1), x1(1)], .15)
    hold off
    %quiver(x(1,i), x(2,i), -x1(1),-x1(2))
    axis([5 11 -1 1])
    drawnow
end

% figure(3)
% z = zeros(size(x(1,:)));
% surface([x(1,:);x(1,:)],[x(2,:);x(2,:)],[z;z],[t;t],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',2);
% colorbar

%% try to plot drone image
% figure(3)
% 
% M = imread('drone.png');
% J = imrotate(M,35, 'bilinear' ,'crop');
% 
% %imagesc(M);
% % imshow('drone.png');
% imagesc('XData', [-10 10],'YData', [10 -10],'CData', J);
% %imshow(M, 'XData', [-180 180],'YData', [90 -90])


%% Trying somthing else now

%% Define design parameters
% figure
% D2R = pi/180;
% R2D = 180/pi;
% b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
% a   = b/3;   % the legth of small square base of quadcopter(b/4)
% H   = 0.06;  % hight of drone in Z direction (4cm)
% H_m = H+H/2; % hight of motor in z direction (5 cm)
% r_p = b/4;   % radius of propeller
% ro = 45*D2R;                   % angle by which rotate the base of quadcopter
% Ri = [cos(ro) -sin(ro) 0;
%       sin(ro) cos(ro)  0;
%        0       0       1];     % rotation matrix to rotate the coordinates of base 
% base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
%            -a/2 -a/2 a/2 a/2;
%              0    0   0   0];
% base = Ri*base_co;             % rotate base Coordinates by 45 degree 
% 
% % design the base square
%  drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
%  drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
%  alpha(drone(1:2),0.7);
%  
%  combinedobject = hgtransform('parent',gca );
%  set(drone,'parent',combinedobject)

%% simulate quadrotor using piccard iteration
% clear
% 
% x0 = [0,0,0,0,0,0]';
% 
% dt = .05;
% time = 10;
% x(:,1) = x0;
% t = 0:dt:time-dt;
% 
% for i = 2:(time/dt)
%     
%     u = controller(x(:,i-1),5);
%     [ubar, lambda(i),h(i)] = barrier(x(:,i-1),u);
%     x(:,i) = x(:,i-1)+qdynamics(x(:,i-1), u)*dt;
% 
% end

%% animate trajectory
% record video
myVideo = VideoWriter('hoverController2'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

figure(2)
for i = 1:length(x)
    plot(x(1,1:i), x(2,1:i))
    hold on
%     plot(10*ones(1,20),linspace(-1,1,20))
    hold off
    theta = 1*x(3,i);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    x1 = R*[0;1];
%     y1 = R*[1;0];
    hold on
    %quiver([x(1,i), x(1,i)], [x(2,i),x(2,i)], [x1(1), x1(2)],[x1(2), x1(1)])
    quiver([x(1,i),x(1,i)], [x(2,i), x(2,i)], [x1(2), -x1(2)],[-x1(1), x1(1)], .15)
    hold off
    %quiver(x(1,i), x(2,i), -x1(1),-x1(2))
    axis([0 20 -.5 .5 ])
    xlabel('x (m)')
    ylabel('y (m)')
    title('Accelerating to 5m/s')
    pause(.1)
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
%     drawnow
end

