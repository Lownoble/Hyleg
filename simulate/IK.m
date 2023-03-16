function [theta1,theta2] = IK(x,y)
%Inverse Kinematics
%逆运动学解算
syms a b;
equ = [x == 300*cos(a)-300*cos(pi/2-b),y == -300*sin(a)-300*sin(pi/2-b)];
answ = solve(equ,[a,b]);
if double(answ.a(1))<pi && double(answ.a(1))>0
    theta1 = double(answ.a(1));
elseif double(answ.a(2))<pi && double(answ.a(2))>0
    theta1 = double(answ.a(2));
end
if double(answ.b(1))<pi && double(answ.b(1))>0
    theta2 = double(answ.b(1));
elseif double(answ.b(2))<pi && double(answ.b(2))>0
    theta2 = double(answ.b(2));
end

end