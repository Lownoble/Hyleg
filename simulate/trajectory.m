INIT_ANGLE1= 45.0/180*pi;
INIT_ANGLE2= 45.0/180*pi;

H = 540;
HF = 50;
LF = 100;
LB = 100;
T = 100;
Ts = 2;
BOTH_RATIO = 1.2;
v = (LB+LF)/(T*BOTH_RATIO);

R = 50;

square_length = 100;
square_width = 100;

stand_trajectory = [];
swing_trajectory = [];

%% 

for i = 0:(T*BOTH_RATIO)

   stand_x = (LB+LF)/(T*BOTH_RATIO)*((T*BOTH_RATIO)-i)-LB;
    stand_y = -H;

%     	theta = 1.0*pi*i/T;
% 	    stand_x = R - R*cos(theta);
% 	    stand_y = -H + R*sin(theta);
% 
% 
%        if(i<T/2)
% 	    	stand_x = 0; stand_y = -H + 2.0*square_width*i/T;
%        else
% 	    	stand_x = 2.0*square_length*(i-T/2)/T; stand_y = -H + square_width;
%        end

    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    stand_trajectory = [stand_trajectory; stand_theta1-INIT_ANGLE1,stand_theta2-INIT_ANGLE2];
end

for i = 0:T

    swing_x = -6*(v*T+LB+LF)/(T^5)*(T-i)^5+15*(v*T+LB+LF)/(T^4)*(T-i)^4-10*(v*T+LB+LF)/(T^3)*(T-i)^3+v*(T-i)+LF;
    if(i<50)
            swing_y = -H + 2*HF*(i/T-0.25/pi*sin(4*pi*i/T));
    else
            swing_y = -H +2*HF*(1-i/T+0.25/pi*sin(4*pi*i/T));
    end

%     	theta = 1.0*pi*i/T;
% 	    swing_x = R + R*cos(theta);
% 	    swing_y = -H - R*sin(theta);

%        if(i<T/2)
% 	    	swing_x = square_length; swing_y = -H + square_width -2.0*square_width*i/T;
%        else
% 	    	swing_x = square_length - 2.0*square_length*(i-T/2)/T; swing_y = -H;
%        end

    [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    swing_trajectory = [swing_trajectory; swing_theta1-INIT_ANGLE1,swing_theta2-INIT_ANGLE2];
end


%% circle

for i = 0:T


    	theta = 1.0*pi*i/T;
	    stand_x = R - R*cos(theta);
	    stand_y = -H + R*sin(theta);

	    swing_x = R + R*cos(theta);
	    swing_y = -H - R*sin(theta);

    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    stand_trajectory = [stand_trajectory; stand_theta1,stand_theta2];
     [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    swing_trajectory = [swing_trajectory; swing_theta1,swing_theta2];
end

%% square

for i = 0:T


       if(i<T/2)
	    	stand_x = 0; stand_y = -H + square_width*(2*i/T-0.5/pi*sin(4*pi*i/T));
       else
	    	stand_x = square_length*((2*i-T)/T-0.5/pi*sin(2*pi*(2*i-T)/T)); stand_y = -H + square_width;
       end

       if(i<T/2)
	    	swing_x = square_length; swing_y = -H + square_width *((T-2*i)/T+0.5/pi*sin(4*pi*i/T));
       else
	    	swing_x = square_length - square_length*((2*i-T)/T-0.5/pi*sin(2*pi*(2*i-T)/T)); swing_y = -H;
       end

    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    stand_trajectory = [stand_trajectory; stand_theta1,stand_theta2];
     [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    swing_trajectory = [swing_trajectory; swing_theta1,swing_theta2];
end

%% 摆线

for i = 0:T

    stand_x = (LB+LF)*((T-i)/T+0.5/pi*sin(2*pi*i/T))-LB;
    stand_y = -H;
    swing_x = (LB+LF)*(i/T-0.5/pi*sin(2*pi*i/T))-LB;
    if(i<T/2)
            swing_y = -H + 2*HF*(i/T-0.25/pi*sin(4*pi*i/T));
    else
            swing_y = -H +2*HF*(1-i/T+0.25/pi*sin(4*pi*i/T));
    end

    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    stand_trajectory = [stand_trajectory; stand_theta1,stand_theta2];
     [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    swing_trajectory = [swing_trajectory; swing_theta1,swing_theta2];
end


