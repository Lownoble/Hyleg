clear all
clc

%% 导入数据
data1 = walk_dataInput('./DATA/walk_0.0MPa_0kg.txt');
data2 = walk_dataInput('./DATA/walk_0.2MPa_0kg.txt');
data3 = walk_dataInput('./DATA/walk_0.5MPa_0kg.txt');

data4 = walk_dataInput('./DATA/walk_0.0MPa_5kg.txt');
data5 = walk_dataInput('./DATA/walk_0.2MPa_5kg.txt');
data6 = walk_dataInput('./DATA/walk_0.5MPa_5kg.txt');

data7 = walk_dataInput('./DATA/walk_0.2MPa_10kg.txt');
data8 = walk_dataInput('./DATA/walk_0.5MPa_10kg.txt');
data9 = walk_dataInput('./DATA/walk_0.5MPa_15kg.txt');

INIT_ANGLE1= 45.0/180*pi;
INIT_ANGLE2= 45.0/180*pi;
data4(1,24) = data4(1,24)-36;

%% 电流处理
j = 19;
data1 = current_process(data1,j);
data2 = current_process(data2,j);
data3 = current_process(data3,j);
data4 = current_process(data4,j);
data5 = current_process(data5,j);
data6 = current_process(data6,j);
data7 = current_process(data7,j);
data8 = current_process(data8,j);
data9 = current_process(data9,j);

%% plot

figure(1)
i = 1:400;
t = i/100;
Z = zeros(600,1);
plot(t,-data4(i,14));
hold on
plot(t,-data1(i,j),'--','Color',[1 0.3 0.3],'LineWidth',1.2,'HandleVisibility','off');
hold on
plot(t,-data2(i,j),'--','Color',[0.2 0.4 0.8],'LineWidth',1.2,'HandleVisibility','off');
hold on
plot(t,-data3(i,j),'--','Color',[0.3 0.8 0.2],'LineWidth',1.2,'HandleVisibility','off');
hold on
for d4 = 1:350
    data4(d4,j) = data4(d4+294,j);
end
plot(t,-data4(i+294,j),'Color',[1 0.3 0.3],'LineWidth',1.2);
hold on
plot(t,-data5(i,j),'Color',[0.2 0.4 1],'LineWidth',1.2);
hold on
plot(t,-data6(i,j),'Color',[0.3 0.8 0.2],'LineWidth',1.2);
% hold on
xlabel("时间(s)")
ylabel("电流(A)")
xlim([0 4])
ylim([-15 30])
% legend("0MPa","0.2MPa","0.5MPa")
set(gcf, 'Position', [100, 100, 800, 400]);


%% cal current
sum = 0;
[N,M] = size(data9);
for i = 1:N
    sum = sum+abs(data9(i,j));
end
averange = sum/N;
