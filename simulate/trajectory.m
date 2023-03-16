H = 500;
LF = 100;
LB = 100;
A = -0.01;T = 100;
stand_trajectory = [];
swing_trajectory = [];

for times = 0:100
    stand_x = -(LB+LF)/T*times+LF;
    stand_y = -H;
    swing_x = (LB+LF)/T*times-LF;
    swing_y = A*(swing_x-LF)*(swing_x+LB)-H;
    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    stand_trajectory = [stand_trajectory; stand_theta1,stand_theta2];
    swing_trajectory = [swing_trajectory; swing_theta1,swing_theta2];
end