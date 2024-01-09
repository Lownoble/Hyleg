clc
clear

data1 = csvread("./data_stanceF/position+vel_v0_3.csv",1,0);
data2 = csvread("./data_stanceF/position+vel+feedback_v0_2.csv",1,0);
data3 = csvread("./data_stanceF/position+vel+feedback_v0_kd08_2.csv",1,0);
% for i=1:4
%     motorState_q(:,i) = data(:,2+i*4); motorState_qd(:,i) = data(:,3+i*4); motorState_i(:,i) = data(:,4+i*4);
%     motorCmd_q(:,i) = data(:,18+i*4); motorCmd_qd(:,i) = data(:,19+i*4); motorCmd_tau(:,i) = data(:,20+i*4);
% end
i = 1:580;
position_d(:,1:2) = (data1(i,33:34)+data2(i,33:34)+data3(i,33:34))/3;
position_d(:,3:4) = (data1(i,37:38)+data2(i,37:38)+data3(i,37:38))/3;
position_f1 = data1(i+1,[35,36,39,40]);
position_f2 = data2(:,[35,36,39,40]);
position_f3 = data3(i+1,[35,36,39,40]);
i = 1:500;
figure(1)
a = plot(i/100,position_d(i,4),'g','LineWidth',2);
a.Color(4) = 0.3;
hold on
plot(i/100,position_f1(i,4),'Color',[0.4 0.4 1],'LineWidth',1.2);
hold on
plot(i/100,position_f3(i,4),'Color',[1 0.3 0.3],'LineWidth',1.2);
legend("目标位置","位控实际位置","力控实际位置");
xlim([2 4])
ylim([-550 -450])
xlabel("时间(s)");
ylabel("Z轴坐标(mm)");

figure(2)
j=1;
while(j<520)
    while(position_d(j,2)==-540)
            contact(j) = 0;
            j=j+1;
    end
    while(position_d(j,4)==-540)
            contact(j) = 1;
            j=j+1;
    end
end

for j=1:500
    Hd(j) = 540;
    if(contact(j)==0)
        H1(j) = -position_f1(j+45,4);
        H2(j) = -position_f2(j+45,4);
        H3(j) = -position_f3(j+45,4);
    end
    if(contact(j)==1)
        H1(j) = -position_f1(j,4);
        H2(j) = -position_f2(j,4);
        H3(j) = -position_f3(j,4);
    end
end
plot(i/100,Hd-H1,'Color',[0.3 0.4 1],'LineWidth',1.2)
hold on
plot(i/100,Hd-H3,'Color',[1 0.3 0.3],'LineWidth',1.2)
legend("位控位置误差","力控位置误差");
xlim([2 4])
ylim([-5 15])
xlabel("时间(s)");
ylabel("机身高度误差(mm)");



%% 
data = load("data_stanceF\F_a0001_b06.txt");
contact = data(:,1:2);
FL = data(:,3:4);
FR = data(:,5:6);
tau = data(:,7:10);
i = 1:200;
plot(-FL(i,2),'r','LineWidth',2)
hold on
plot(-FR(i,2),'b','LineWidth',2)
hold on
plot(contact(i,:)*10-20)
legend("左腿输出力","右腿输出力");