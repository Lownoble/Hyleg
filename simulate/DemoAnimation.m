clc
clear

%% load
%%Time normalT0 normalT1 contact0 contact1 phase Q Qd position velocity vCmd feetpos feetvel

data = load("swich.txt");

time = data(:,1);
normalT = data(:,2:3);
contact = data(:,4:5);
phase = data(:,6);
Q = data(:,7:10);
Qd = data(:,11:14);
position = data(:,15:16)*1000;
velocity = data(:,17:18);
vCmd = data(:,19);
nextStep = vCmd*1.2*1000/2-position;
feetPos = data(:,21:24);
feetVel = data(:,25:27);
plot(time,Q);
legend("Q1","Q2","Q3","Q4");

%% 
[N,M] = size(time);
start_point = 0;
foot_L = [];
foot_R = [];
pic_num = 1;
for i = 2:N
    if(phase(i)-phase(i-1)<-0.5) start_point = start_point+position(i-1)+-position(i);end
    body_point = [position(i)+start_point,0];
    knee_point_L = [-300*sin(Q(i,1)),-300*cos(Q(i,1))] + body_point;
    knee_point_R = [-300*sin(Q(i,3)),-300*cos(Q(i,3))] + body_point;
    foot_point_L = [-300*sin(Q(i,1)) + 300*cos(Q(i,1)+Q(i,2)), -300*cos(Q(i,1)) - 300*sin(Q(i,1)+Q(i,2))] + body_point;
    foot_point_R = [-300*sin(Q(i,3)) + 300*cos(Q(i,3)+Q(i,4)), -300*cos(Q(i,3)) - 300*sin(Q(i,3)+Q(i,4))] + body_point;
    foot_L = [foot_L;foot_point_L];
    foot_R = [foot_R;foot_point_R];
    figure(1)
    axis equal
    plot(foot_L(:,1),foot_L(:,2),'b','LineWidth',1);
    hold on  
    plot(foot_R(:,1),foot_R(:,2),'R','LineWidth',1);
    hold on 
    plot([body_point(1),knee_point_L(1)],[body_point(2),knee_point_L(2)],'b','LineWidth',2);
    hold on
    plot([foot_point_L(1),knee_point_L(1)],[foot_point_L(2),knee_point_L(2)],'b','LineWidth',2);
    hold on
    plot([body_point(1),knee_point_R(1)],[body_point(2),knee_point_R(2)],'r','LineWidth',2);
    hold on
    plot([foot_point_R(1),knee_point_R(1)],[foot_point_R(2),knee_point_R(2)],'r','LineWidth',2);
    hold on
    plot((nextStep(i)+body_point(1)),-540,'*k');
    hold on
    axis equal
    xlim([-300 1000])
    ylim([-600 100]) 

    drawnow;
    F = getframe(figure(1));
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'swich.gif','gif', 'Loopcount',inf,'DelayTime',0.02);
    else
        imwrite(I,map,'swich.gif','gif','WriteMode','append','DelayTime',0.02);
    end
    pic_num = pic_num + 1;


    hold off
end