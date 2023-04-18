H = 500;
HF = 100;
LF = 100;
LB = 100;
T = 100;

R = 50;

length = 100;
width = 50;

stand_trajectory = [];
swing_trajectory = [];

for i = 0:100

%     stand_x = (LB+LF)*((T-i)/T+0.5/pi*sin(2*pi*i/T))-LB;
%     stand_y = -H;
%     swing_x = (LB+LF)*(i/T-0.5/pi*sin(2*pi*i/T))-LB;
%     if(i<50)
%             swing_y = -H + 2*HF*(i/T-0.25/pi*sin(4*pi*i/T));
%     else
%             swing_y = -H +2*HF*(1-i/T+0.25/pi*sin(4*pi*i/T));
%     end

%     	theta = 1.0*pi*i/T;
% 	    stand_x = R - R*cos(theta);
% 	    stand_y = -H + R*sin(theta);
% 	    swing_x = R + R*cos(theta);
% 	    swing_y = -H - R*sin(theta);

       if(i<T/2)
	    	stand_x = 0; stand_y = -H + 2.0*width*i/T;
	    	swing_x = length; swing_y = -H + width -2.0*width*i/T;
       else
	    	stand_x = 2.0*length*(i-T/2)/T; stand_y = -H + width;
	    	swing_x = length - 2.0*length*(i-T/2)/T; swing_y = -H;
       end

    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    stand_trajectory = [stand_trajectory; stand_theta1,stand_theta2];
    swing_trajectory = [swing_trajectory; swing_theta1,swing_theta2];
end

