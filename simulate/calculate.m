clc
clear

%% FK
theta1 = 45/180*pi;
theta2 = 90/180*pi;

hip_x = -60 * sin(theta1-theta2);
hip_y = -60 * cos(theta1-theta2);

knee_x = -300 * sin(theta1);
knee_y = -300 * cos(theta1);

foot_x = knee_x + 5*hip_x;
foot_y = knee_y + 5*hip_y;


Cylinder_len = 2*60*cos(theta2) + 2*60*cos(pi/2-theta1);

axis equal
plot([0,knee_x],[0,knee_y],'r','LineWidth',3);
hold on 
plot([knee_x,foot_x],[knee_y,foot_y],'r','LineWidth',3);
hold on 
xlim([-300 300])
ylim([-500 100])

%% IF
x = 0;
y = -450;
[theta1,theta2]=IK(x,y);
hip_x = -60 * sin(theta1-theta2);
hip_y = -60 * cos(theta1-theta2);

knee_x = -300 * sin(theta1);
knee_y = -300 * cos(theta1);

foot_x = knee_x + 5*hip_x;
foot_y = knee_y + 5*hip_y;
axis equal
plot([0,knee_x],[0,knee_y],'r','LineWidth',3);
hold on 
plot([knee_x,foot_x],[knee_y,foot_y],'r','LineWidth',3);
hold on 
xlim([-300 300])
ylim([-500 100])


%% trajectory
load("stand_trajectory.mat");
load("swing_trajectory.mat");

% H = 400;
% LF = 100;
% LB = 100;
% A = -0.01;T = 100;
% stand_trajectory = [];
% swing_trajectory = [];
% 
% for times = 0:100
%     stand_x = -(LB+LF)/T*times+LF;
%     stand_y = -H;
%     swing_x = (LB+LF)/T*times-LF;
%     swing_y = A*(swing_x-LF)*(swing_x+LB)-H;
%     [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
%     [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
%     stand_trajectory = [stand_trajectory; stand_theta1,stand_theta2];
%     swing_trajectory = [swing_trajectory; swing_theta1,swing_theta2];
% end
%% plot1
stand_hip_trajectory = [];
stand_knee_trajectory = [];
stand_foot_trajectory = [];
swing_hip_trajectory = [];
swing_knee_trajectory = [];
swing_foot_trajectory = [];
pic_num = 1;
for i = 1:T
    theta1 = stand_trajectory(i,1);
    theta2 = stand_trajectory(i,2);
    hip_x = -60 * sin(theta1-theta2);
    hip_y = -60 * cos(theta1-theta2);
    knee_x = -300 * sin(theta1);
    knee_y = -300 * cos(theta1);
    foot_x = knee_x + 5*hip_x;
    foot_y = knee_y + 5*hip_y;
    stand_hip_trajectory = [stand_hip_trajectory; hip_x,hip_y];
    stand_knee_trajectory = [stand_knee_trajectory; knee_x,knee_y];
    stand_foot_trajectory = [stand_foot_trajectory; foot_x,foot_y];
    
    theta1 = swing_trajectory(i,1);
    theta2 = swing_trajectory(i,2);
    hip_x = -60 * sin(theta1-theta2);
    hip_y = -60 * cos(theta1-theta2);
    knee_x = -300 * sin(theta1);
    knee_y = -300 * cos(theta1);
    foot_x = knee_x + 5*hip_x;
    foot_y = knee_y + 5*hip_y;
    swing_hip_trajectory = [swing_hip_trajectory; hip_x,hip_y];
    swing_knee_trajectory = [swing_knee_trajectory; knee_x,knee_y];
    swing_foot_trajectory = [swing_foot_trajectory; foot_x,foot_y];


    figure(1)
    axis equal
    plot(stand_foot_trajectory(:,1),stand_foot_trajectory(:,2),'r','LineWidth',2);
    hold on
    plot(swing_foot_trajectory(:,1),swing_foot_trajectory(:,2),'g','LineWidth',2);
    hold on
    axis equal
    xlim([-300 100])
    ylim([-500 100]) 

    plot([0,stand_knee_trajectory(i,1)],[0,stand_knee_trajectory(i,2)],'k','LineWidth',3);
    hold on 
    plot([stand_foot_trajectory(i,1),stand_knee_trajectory(i,1)],[stand_foot_trajectory(i,2),stand_knee_trajectory(i,2)],'k','LineWidth',3);
    hold on 

    plot([0,swing_knee_trajectory(i,1)],[0,swing_knee_trajectory(i,2)],'k','LineWidth',3);
    hold on 
    plot([swing_foot_trajectory(i,1),swing_knee_trajectory(i,1)],[swing_foot_trajectory(i,2),swing_knee_trajectory(i,2)],'k','LineWidth',3);

    drawnow;
    F = getframe(figure(1));
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'test.gif','gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pic_num = pic_num + 1;
    hold off
end

%% plot2
stand_knee_trajectory = [];
stand_foot_trajectory = [];
swing_knee_trajectory = [];
swing_foot_trajectory = [];
for i = 1:(T*BOTH_RATIO)
    stand_theta1 = stand_trajectory(i,1);
    stand_theta2 = stand_trajectory(i,2);
    stand_knee_trajectory = [stand_knee_trajectory; -300*sin(stand_theta1),-300*cos(stand_theta1)];
    stand_foot_trajectory = [stand_foot_trajectory; -300*sin(stand_theta1)+300*cos(stand_theta2),-300*cos(stand_theta1)-300*sin(stand_theta2)];
    figure(1)
    plot(stand_foot_trajectory(:,1),stand_foot_trajectory(:,2),'r','LineWidth',3);
    hold on
    xlim([-80 80])
    ylim([-550 -380])

    figure(2)
    axis equal

    plot([0,stand_knee_trajectory(i,1)],[0,stand_knee_trajectory(i,2)],'r','LineWidth',3);
    hold on 
    plot([stand_foot_trajectory(i,1),stand_knee_trajectory(i,1)],[stand_foot_trajectory(i,2),stand_knee_trajectory(i,2)],'r','LineWidth',3);
    hold on 
    xlim([-300 300])
    ylim([-550 100])
    hold off
end



for i = 1:T

    swing_theta1 = swing_trajectory(i,1);
    swing_theta2 = swing_trajectory(i,2);

    swing_knee_trajectory = [swing_knee_trajectory; -300*sin(swing_theta1),-300*cos(swing_theta1)];
    
    swing_foot_trajectory = [swing_foot_trajectory; -300*sin(swing_theta1)+300*cos(swing_theta2),-300*cos(swing_theta1)-300*sin(swing_theta2)];
    figure(1)
%     plot(stand_knee_trajectory(:,1),stand_knee_trajectory(:,2),'r','LineWidth',3);
%     hold on

%     plot(swing_knee_trajectory(:,1),swing_knee_trajectory(:,2),'g','LineWidth',3);
%     hold on
    plot(swing_foot_trajectory(:,1),swing_foot_trajectory(:,2),'g','LineWidth',3);
    hold on

    figure(2)
    axis equal

    plot([0,swing_knee_trajectory(i,1)],[0,swing_knee_trajectory(i,2)],'g','LineWidth',3);
    hold on 
    plot([swing_foot_trajectory(i,1),swing_knee_trajectory(i,1)],[swing_foot_trajectory(i,2),swing_knee_trajectory(i,2)],'g','LineWidth',3);

    xlim([-300 300])
    ylim([-550 100])
    hold off


end

%% 
fid=fopen(['C.txt'],'w');%写入文件路径
[r,c]=size(swing_trajectory);            % 得到矩阵的行数和列数
 for i=1:r
  fprintf(fid,'{%f,\t',swing_trajectory(i,1));
  fprintf(fid,'%f},\r\n',swing_trajectory(i,2));
 end
fclose(fid);
%% 

syms syms theta1 theta2 x y;
equ = [x == -300*sin(theta1)+300*cos(theta2),y == -300*cos(theta1)-300*sin(theta2)];
answ = solve(equ,[theta1,theta2]);

%% 
a = [];
for i = 1:100
    a = [a;stand_trajectory(i,1)-stand_trajectory(i+1,1),stand_trajectory(i,2)-stand_trajectory(i+1,2)];
end



