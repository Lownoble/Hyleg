clear all
clc

%% 导入数据
data1 = textread('./DATA/horizontal_0.0MPa_0kg.txt');
data2 = textread('./DATA/horizontal_0.2MPa_10kg.txt');
data3 = textread('./DATA/horizontal_0.3MPa_10kg.txt');
data4 = textread('./DATA/horizontal_0.4MPa_10kg.txt');
data5 = textread('./DATA/horizontal_0.5MPa_10kg.txt');

INIT_ANGLE1= 45.0/180*pi;
INIT_ANGLE2= 45.0/180*pi;

%% 

T = 132;
H = 540;
long = 200;
stand_trajectory = [];
swing_trajectory = [];

for i = 0:T
		stand_x = long*(1.0*i/T-0.5/pi*sin(2.0*pi*i/T)) - long/2;
		stand_y = -H;

		swing_x = long/2 - long*(1.0*i/T-0.5/pi*sin(2.0*pi*i/T));
		swing_y = -H;

    [stand_theta1,stand_theta2] = IK(stand_x,stand_y);
    stand_trajectory = [stand_trajectory; stand_theta1-INIT_ANGLE1,stand_theta2-stand_theta1+INIT_ANGLE1-INIT_ANGLE2];
     [swing_theta1,swing_theta2] = IK(swing_x,swing_y);
    swing_trajectory = [swing_trajectory; swing_theta1-INIT_ANGLE1,swing_theta2-swing_theta1+INIT_ANGLE1-INIT_ANGLE2];


end

%% 电流

sum1 = 0;sum2 = 0;sum3 = 0;sum4 = 0;sum5 = 0;

for i = 1:length(data1)
     if(data1(i,7)<-5 && data1(i,2)==0&&data1(i,1)<78)       data1(i,7) = data1(i,7)+36;end
     if(data1(i,7)>10 && data1(i,2)==1)       data1(i,7) = data1(i,7)-36;end
     if(data1(i,7)<-10 && data1(i,2)==1&&data1(i,1)>60)       data1(i,7) = data1(i,7)+36;end
    sum1 = sum1+ data1(i,7)^2;
end
average1 = sqrt(sum1/length(data1));

for i = 1:length(data2)
    if(data2(i,7)<-5&& data2(i,2)==0)       data2(i,7) = data2(i,7)+36;end
    sum2 = sum2+ data2(i,7)^2;
end
average2 = sqrt(sum2/length(data2));

for i = 1:length(data3)
    if(data3(i,7)<-5&& data3(i,2)==0)       data3(i,7) = data3(i,7)+36;end
    sum3 = sum3+ data3(i,7)^2;
end
average3 = sqrt(sum3/length(data3));

for i = 1:length(data4)
    if(data4(i,7)<-5&& data4(i,2)==0)       data4(i,7) = data4(i,7)+36;end
    sum4 = sum4+ data4(i,7)^2;
end
average4 = sqrt(sum4/length(data4));

for i = 1:length(data5)
    if(data5(i,7)<-5&& data5(i,2)==0)       data5(i,7) = data5(i,7)+36;end
    sum5 = sum5+ data5(i,7)^2;
end
average5 = sqrt(sum5/length(data5));

figure(1)
plot(data1(1:700,7));
hold on
plot(data2(1:700,7));
hold on
plot(data3(1:700,7));
hold on
plot(data4(1:700,7));
hold on
plot(data5(1:700,7));
hold on
title("负载5KG")
ylabel("电流(A)")
legend("0MPa","0.2MPa","0.3MPa","0.4MPa","0.5MPa")


%% 位置

a = 5;
theta = [];
for i = 1:4
 theta = [theta;swing_trajectory];
 theta = [theta;stand_trajectory];

end


figure(2)
plot(theta(1:700,1));
hold on
plot(data1(1:700,a));
hold on
% plot(data2(1:700,a-1));
% hold on
plot(data2(1:700,a));
hold on
plot(data3(1:700,a));
hold on
% plot(data4(1:700,a));
% hold on
% plot(data5(1:700,a));
% hold on
title("气缸0.2MPa")
ylabel("角度(rad)")
legend("理想","0kg","5kg","10kg")

%% 轨迹

trajectory1 = [];trajectory2 = [];trajectory3 = [];trajectory4 = [];trajectory5 = [];

for i = 1: 700

    [trajectory1(i,1),trajectory1(i,2)] = FK(data2(i,5)+INIT_ANGLE1,data2(i,10)+data2(i,5)+INIT_ANGLE2);
    [trajectory1(i,3),trajectory1(i,4)] = FK(theta(i,1)+INIT_ANGLE1,theta(i,2)+theta(i,1)+INIT_ANGLE2);


end

figure(3)
plot(trajectory1(:,1),trajectory1(:,2),'k');
hold on
plot(trajectory1(:,3),trajectory1(:,4),'r','LineWidth',2);




