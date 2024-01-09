function [x,z] = calcPEe2H(q1,q2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    l1 = 0.300;
    l2 = 0.300;

    x = -l1*sin(q1) + l2*cos(q1+q2);

    z = -l1*cos(q1) - l2*sin(q1+q2);

end