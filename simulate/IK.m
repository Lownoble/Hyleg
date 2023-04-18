function [theta1,theta2] = IK(x,y)
%Inverse Kinematics
%逆运动学解算

% syms a b;
% equ = [x == -300*sin(a)+300*cos(b),y == -300*cos(a)-300*sin(b)];
% answ = solve(equ,[a,b]);

a1 = 2*atan(((600*x*(600*y + (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2)))/(x^2 + 600*x + y^2) - 600*y - 600*x + (x^2*(600*y + (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2)))/(x^2 + 600*x + y^2) + (y^2*(600*y + (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2)))/(x^2 + 600*x + y^2))/(x^2 + y^2 - 600*y));
% a2 = 2*atan(((600*x*(600*y - (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2)))/(x^2 + 600*x + y^2) - 600*y - 600*x + (x^2*(600*y - (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2)))/(x^2 + 600*x + y^2) + (y^2*(600*y - (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2)))/(x^2 + 600*x + y^2))/(x^2 + y^2 - 600*y));
b1 = -2*atan((600*y + (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2))/(x^2 + 600*x + y^2));
% b2 = -2*atan((600*y - (-(x^2 + y^2)*(x^2 + y^2 - 360000))^(1/2))/(x^2 + 600*x + y^2));
 
theta1 = a1;
theta2 = b1;

% if double(answ.a(1))<pi/2 && double(answ.a(1))>0
%     theta1 = double(answ.a(1));
% elseif double(answ.a(2))<pi/2 && double(answ.a(2))>0
%     theta1 = double(answ.a(2));
% end
% if double(answ.b(1))<pi/2 && double(answ.b(1))>0
%     theta2 = double(answ.b(1));
% elseif double(answ.b(2))<pi/2 && double(answ.b(2))>0
%     theta2 = double(answ.b(2));
% end

end