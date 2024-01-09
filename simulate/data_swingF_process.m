clc
clear
data1 = csvread("./data_swingF/position+feedback_K0.csv",1,0);
data2 = csvread("./data_swingF/position+feedback_K40.csv",1,0);
data3 = csvread("./data_swingF/position+feedback_K80.csv",1,0);
for i=1:4
    motorState_q1(:,i) = data1(:,2+i*4); motorState_qd1(:,i) = data1(:,3+i*4); motorState_i1(:,i) = data1(:,4+i*4);
    motorCmd_q1(:,i) = data1(:,18+i*4); motorCmd_qd1(:,i) = data1(:,19+i*4); motorCmd_tau1(:,i) = data1(:,20+i*4);
end

i = 1:400; 

position_d = (data1(i,37:38)+data2(i,37:38)+data3(i,37:38))/3;
position_f1 = data1(:,39:40);
position_f2 = data2(:,39:40);
position_f3 = data3(:,39:40);

for i=3:410
    position_f1(i,:) = (position_f1(i-2,:)+position_f1(i-1,:)+position_f1(i,:)+position_f1(i+1,:)+position_f1(i+2,:))/5;
    position_f2(i,:) = (position_f2(i-2,:)+position_f2(i-1,:)+position_f2(i,:)+position_f2(i+1,:)+position_f2(i+2,:))/5;
    position_f3(i,:) = (position_f3(i-2,:)+position_f3(i-1,:)+position_f3(i,:)+position_f3(i+1,:)+position_f3(i+2,:))/5;
end
i = 1:400;
error_position1 = abs(position_d(i,:) - position_f1(i+2,:));
error_position2 = abs(position_d(i,:) - position_f2(i+1,:));
error_position3 = abs(position_d(i,:) - position_f3(i+1,:));

figure(1)
a = plot(i/100,position_d(i,2),'g','LineWidth',2);
a.Color(4) = 0.3;
hold on
plot(i/100,position_f1(i+2,2),'Color',[0.4 0.4 1],'LineWidth',1.2);
hold on
plot(i/100,position_f2(i+2,2),'Color',[0.8 0.6 0.2],'LineWidth',1.2);
hold on
plot(i/100,position_f3(i+1,2),'Color',[1 0.3 0.3],'LineWidth',1.2);
legend("目标位置","K=0 实际位置","K=40实际位置","K=80实际位置");
xlim([2 4])
ylim([-550 -450])
xlabel("时间(s)");
ylabel("Z轴坐标(mm)");

figure(2)
plot(i/100,error_position1(i,2));
hold on
plot(i/100,error_position2(i,2));
hold on
plot(i/100,error_position3(i,2));
hold on
legend("K=0","K=40","K=80");

%% 
clc
clear

data = csvread("./data_swingF/sqaure_KP0_KD0.csv",1,0);
for i=1:4
    motorState_q(:,i) = data(:,2+i*4); motorState_qd(:,i) = data(:,3+i*4); motorState_i(:,i) = data(:,4+i*4);
    motorCmd_q(:,i) = data(:,18+i*4); motorCmd_qd(:,i) = data(:,19+i*4); motorCmd_tau(:,i) = data(:,20+i*4);
end

position_ld = data(:,39:40);
position_lf = data(:,41:42);

position_rd = data(:,43:44);
position_rf = data(:,45:46);

figure(1)
plot(motorCmd_q(:,3));
hold on
plot(motorState_q(:,3));

figure(2)
plot(position_rd(:,1),position_rd(:,2),'g','LineWidth',2)
hold on
plot(position_rf(:,1),position_rf(:,2),'k')

axis equal
xlim([-60 60])
ylim([-550 -430])