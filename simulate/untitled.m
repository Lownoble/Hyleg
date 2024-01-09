clc
clear

%% calFootPos

bodyVelGlobal = 0.02;
vGoalGlobal = 0.02;
Tswing = 0.6;
Tstance = 0.6;
kx = 0.005;
phase = 0.866353;
nextStep = bodyVelGlobal*(1-phase)*Tswing + bodyVelGlobal*Tstance/2 + kx*(bodyVelGlobal - vGoalGlobal);

x = 0.1; y = -0.45;
[q1,q2] = IK(x*1000,y*1000);
q2 = q2-q1;
[x1,z1] = calcPEe2H(0.5970,0.3760);
[x2,z2] = calcPEe2H(0.5930,0.36);
%% 
%%Time normalT0 normalT1 contact0 contact1 phase Q Qd position velocity vCmd nextstep feetpos feetvel

data = load("data_swingtest\walking_v0_1.txt");

time = data(:,1);
normalT = data(:,2:3);
contact = data(:,4:5);
phase = data(:,6);
Q = data(:,7:10);
Qd = data(:,11:14);
position = data(:,15:16);
velocity = data(:,17:18);
vCmd = data(:,19);
nextStep = data(:,20);
feetPos = data(:,21:24);
feetVel = data(:,25:28);
% 
% plot(time,feetPos(:,2));
% hold on
% plot(time,feetPos(:,4));
% plot(time,nextStep);
plot(time,Q);
legend("Q1","Q2","Q3","Q4");
% plot(time,position(:,1));
% plot(time,contact(:,1));

%% 
data = load("simulatedata.txt");
[N,M] = size(data);
posbody = data(:,1:2);
velbody = data(:,3:4);
posfoot = data(:,5:6);
velfoot = data(:,7:8);
posfoot2B = posfoot - posbody;
velfoot2B = velfoot - velbody;
Q = data(:,9:12);
QD = data(:,13:16);
PB = data(:,17:18);
VB = data(:,19:20);

figure(1)
plot(posfoot2B);
hold on
plot(velfoot2B);
hold on
plot(PB);
hold on
plot(VB);
legend("px","pz","vx","vz","px","pz","vx","vz");

figure(2)
Qd=[];
for i=1:N
    J = Jacobian(Q(i,1),Q(i,2));
    v = [velfoot2B(i,1);velfoot2B(i,2)];
    qd = inv(J)*v;
    Qd = [Qd;qd(1),qd(2)];
end
plot(QD(:,1:2));
hold on
plot(Qd);
hold on
legend("QD1","QD2","Qd1","Qd2");

figure(3)
vx1=[];vz1=[];
for i=1:N
    J = Jacobian(Q(i,1),Q(i,2));
    qd = [QD(i,1);QD(i,2)];
    v = J*qd;
    vx1 = [vx1;v(1)]; vz1 = [vz1;v(2)];
end
plot(Q(:,1));
hold on
plot(Q(:,2));
hold on
plot(posfoot2B(:,2));
plot(vx1);
hold on
plot(vz1);
legend("Q1","Q2","z","vx","vz");
%% cal F
q1 = 0.4510;
q2 = 0.6687;
J = Jacobian(q1,q2);
F = [0;-100];
tau = J' *F

