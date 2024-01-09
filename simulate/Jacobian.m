function jaco = Jacobian(theta1,theta2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    l1 = 0.3;
    l2 = 0.3;

    s1 = sin(theta1);
    s2 = sin(theta2);

    c1 = cos(theta1);
    c2 = cos(theta2);

    jaco = [0,0;0,0];
    jaco(1, 1) = -l1*c1 - l2*s1*c2 - l2*c1*s2;
    jaco(2, 1) = l1*s1 - l2*c1*c2 + l2*s1*s2;
    jaco(1, 2) = -l2*c1*s2 - l2*s1*c2;
    jaco(2, 2) = l2*s1*s2 - l2*c1*c2;
end