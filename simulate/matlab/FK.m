function [x,y] = FK(theta1,theta2)
%Forword Kinematics
%
% hip_x = -60 * sin(theta1-theta2);
% hip_y = -60 * cos(theta1-theta2);

hip_x = 60 * cos(theta2);
hip_y = -60 * sin(theta2);

knee_x = -300 * sin(theta1);
knee_y = -300 * cos(theta1);

foot_x = knee_x + 5*hip_x;
foot_y = knee_y + 5*hip_y;

x = foot_x;
y = foot_y;
end