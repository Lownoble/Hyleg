clc
clear

%% setting
theta1 = 45/180*pi;
theta2 = 30/180*pi;


%% calculate
hip_x = -60 * cos(pi/2 - theta2);
hip_y = -60 * sin(pi/2 - theta2);

knee_x = 300 * cos(theta1);
knee_y = -300 * sin(theta1);

foot_x = knee_x + 5*hip_x;
foot_y = knee_y + 5*hip_y;

Cylinder_len = 2*60*cos(theta2) + 2*60*cos(pi/2-theta1);


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


%% plot
stand_knee_trajectory = [];
stand_foot_trajectory = [];
swing_knee_trajectory = [];
swing_foot_trajectory = [];
for i = 1:101
    stand_theta1 = stand_trajectory(i,1);
    stand_theta2 = stand_trajectory(i,2);
    swing_theta1 = swing_trajectory(i,1);
    swing_theta2 = swing_trajectory(i,2);
    stand_knee_trajectory = [stand_knee_trajectory; 300*cos(stand_theta1),-300*sin(stand_theta1)];
    swing_knee_trajectory = [swing_knee_trajectory; 300*cos(swing_theta1),-300*sin(swing_theta1)];
    stand_foot_trajectory = [stand_foot_trajectory; 300*cos(stand_theta1)-300*cos(pi/2-stand_theta2),-300*sin(stand_theta1)-300*sin(pi/2-stand_theta2)];
    swing_foot_trajectory = [swing_foot_trajectory; 300*cos(swing_theta1)-300*cos(pi/2-swing_theta2),-300*sin(swing_theta1)-300*sin(pi/2-swing_theta2)];
    figure(1)
    plot(stand_knee_trajectory(:,1),stand_knee_trajectory(:,2),'r','LineWidth',3);
    hold on
    plot(stand_foot_trajectory(:,1),stand_foot_trajectory(:,2),'r','LineWidth',3);
    hold on
    plot(swing_knee_trajectory(:,1),swing_knee_trajectory(:,2),'g','LineWidth',3);
    hold on
    plot(swing_foot_trajectory(:,1),swing_foot_trajectory(:,2),'g','LineWidth',3);
    hold on

    figure(2)
    axis equal
    plot([0,stand_knee_trajectory(i,1)],[0,stand_knee_trajectory(i,2)],'r','LineWidth',3);
    hold on 
    plot([stand_foot_trajectory(i,1),stand_knee_trajectory(i,1)],[stand_foot_trajectory(i,2),stand_knee_trajectory(i,2)],'r','LineWidth',3);
    hold on 
    plot([0,swing_knee_trajectory(i,1)],[0,swing_knee_trajectory(i,2)],'g','LineWidth',3);
    hold on 
    plot([swing_foot_trajectory(i,1),swing_knee_trajectory(i,1)],[swing_foot_trajectory(i,2),swing_knee_trajectory(i,2)],'g','LineWidth',3);

    xlim([-300 300])
    ylim([-500 100])
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

