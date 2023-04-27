clc
close all
clear

%% circle

INIT_ANGLE1= 45.0/180*pi;
INIT_ANGLE2= 38.3/180*pi;

circle_data = textread('./DATA/circle2.txt');
for i = 1:length(circle_data)
    [circle_data(i,9),circle_data(i,10)] = FK(circle_data(i,2)+INIT_ANGLE1,circle_data(i,6)+INIT_ANGLE2);

end

stand_foot_trajectory = [];
swing_foot_trajectory = [];
for i = 1:T
    stand_theta1 = stand_trajectory(i,1);
    stand_theta2 = stand_trajectory(i,2);
    stand_foot_trajectory = [stand_foot_trajectory; -300*sin(stand_theta1)+300*cos(stand_theta2),-300*cos(stand_theta1)-300*sin(stand_theta2)];
    figure(1)
    axis equal
    plot(stand_foot_trajectory(:,1),stand_foot_trajectory(:,2),'g','LineWidth',2);
    hold on
    xlim([-10 110])
    ylim([-510 -390])
end

for i = 1:T
    swing_theta1 = swing_trajectory(i,1);
    swing_theta2 = swing_trajectory(i,2);
    swing_foot_trajectory = [swing_foot_trajectory; -300*sin(swing_theta1)+300*cos(swing_theta2),-300*cos(swing_theta1)-300*sin(swing_theta2)];
    figure(1)
    plot(swing_foot_trajectory(:,1),swing_foot_trajectory(:,2),'g','LineWidth',2);
    hold on
end


plot(circle_data(:,9),circle_data(:,10),'k');


%% line

line_data = textread('./DATA/line1.txt');
for i = 1:length(line_data)
    [line_data(i,9),line_data(i,10)] = FK((line_data(i,2)+INIT_ANGLE1),(line_data(i,6)+INIT_ANGLE2));

end

plot(line_data(:,9),line_data(:,10));


%% square

INIT_ANGLE1= 45.0/180*pi;
INIT_ANGLE2= 51.7/180*pi;
square_data = textread('./DATA/square3.txt');
for i = 1:length(square_data)
    [square_data(i,9),square_data(i,10)] = FK(square_data(i,2)+INIT_ANGLE1,square_data(i,6)+INIT_ANGLE2);

end

stand_foot_trajectory = [];
swing_foot_trajectory = [];
for i = 1:T
    stand_theta1 = stand_trajectory(i,1);
    stand_theta2 = stand_trajectory(i,2);
    stand_foot_trajectory = [stand_foot_trajectory; -300*sin(stand_theta1)+300*cos(stand_theta2),-300*cos(stand_theta1)-300*sin(stand_theta2)];
    figure(1)
    axis equal
    plot(stand_foot_trajectory(:,1),stand_foot_trajectory(:,2),'g','LineWidth',2);
    hold on
    xlim([-20 120])
    ylim([-470 -330])
end

for i = 1:T
    swing_theta1 = swing_trajectory(i,1);
    swing_theta2 = swing_trajectory(i,2);
    swing_foot_trajectory = [swing_foot_trajectory; -300*sin(swing_theta1)+300*cos(swing_theta2),-300*cos(swing_theta1)-300*sin(swing_theta2)];
    figure(1)
    plot(swing_foot_trajectory(:,1),swing_foot_trajectory(:,2),'g','LineWidth',2);
    hold on
end

plot(square_data(:,9),square_data(:,10),'k');


%% trajectory

trajectory_data = textread('./DATA/trajectory1.txt');

for i = 1: (length(trajectory_data))
    [trajectory_data(i,9),trajectory_data(i,10)] = FK(trajectory_data(i,2)+INIT_ANGLE1,trajectory_data(i,6)+INIT_ANGLE2);

end

stand_foot_trajectory = [];
swing_foot_trajectory = [];
for i = 1:(T*BOTH_RATIO)
    stand_theta1 = stand_trajectory(i,1);
    stand_theta2 = stand_trajectory(i,2);
    stand_foot_trajectory = [stand_foot_trajectory; -300*sin(stand_theta1)+300*cos(stand_theta2),-300*cos(stand_theta1)-300*sin(stand_theta2)];
    figure(1)

    plot(stand_foot_trajectory(:,1),stand_foot_trajectory(:,2),'r','LineWidth',3);
    hold on
    xlim([-80 80])
    ylim([-500 -380])
end

for i = 1:T
    swing_theta1 = swing_trajectory(i,1);
    swing_theta2 = swing_trajectory(i,2);
    swing_foot_trajectory = [swing_foot_trajectory; -300*sin(swing_theta1)+300*cos(swing_theta2),-300*cos(swing_theta1)-300*sin(swing_theta2)];
    figure(1)
    plot(swing_foot_trajectory(:,1),swing_foot_trajectory(:,2),'g','LineWidth',3);
    hold on
end

figure(1)
% plot(trajectory_data(:,9),trajectory_data(:,10),'k');

%% TEST
INIT_ANGLE1= 0.0/180*pi
INIT_ANGLE2= 51.7/180*pi
test_data = textread('./DATA/test.txt');

for i = 1: (length(test_data))
    [test_data(i,9),test_data(i,10)] = FK(test_data(i,2)+INIT_ANGLE1,test_data(i,6)+INIT_ANGLE2);

end

figure(1)
plot(test_data(:,9),test_data(:,10),'k');
