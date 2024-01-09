function [x,y] = FK(theta1,theta2)
%Forword Kinematics

l1 = 250;
l2 = 250;

foot_x = -l1*sin(theta1) + l2*cos(theta1+theta2);
foot_y = -l1*cos(theta1) - l2*sin(theta1+theta2);

x = foot_x;
y = foot_y;
end