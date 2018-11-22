clear all
close all
clc

load('gps_data.mat');
load('accel.mat');
load('kistler.mat');
load('lidar.mat');
g = 9.81;

time_acc = accel.Time;
time_acc = time_acc-time_acc(1);
pitch_s = accel.Data(:,8);
roll_s = accel.Data(:,9);
yaw_s = accel.Data(:,10);
acc_x = accel.Data(:,11);
acc_y = accel.Data(:,12);
acc_z = accel.Data(:,13);

time_gps = gps_data.Time;
time_gps = time_gps-time_gps(1);
latitude = gps_data.Data(:,1);
longitude = gps_data.Data(:,2);

time_kistler = kistler.Time;
time_kistler = time_kistler-time_kistler(1);
V_x = kistler.Data(:,4);
V_y = kistler.Data(:,5);
V_z = kistler.Data(:,6);

                        % % ONLY DISPLAY DATA % %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f1 = figure_fullScreen;
subplot(2,4,1)
plot(time_acc,acc_x./g);
grid
title('Longitudinal');
xlabel('t[s]');
ylabel('X[G]');
xlim([0, max(time_acc)]);

subplot(2,4,2)
plot(time_acc,acc_y./g);
grid
title('Lateral');
xlabel('t[s]');
ylabel('Y[G]');
xlim([0, max(time_acc)]);

subplot(2,4,5);
plot(time_acc,acc_z./g);
grid
title('Accel_Z');
xlabel('t[s]');
ylabel('Z[G]');
xlim([0, max(time_acc)]);

subplot(2,4,6);
plot(acc_x./g,acc_y./g);
grid
title('G circle');
xlabel('acc_x[G]');
ylabel('acc_y[G]');
axis equal
x_circle = (-2:1e-2:2);
y_circle_upper = sqrt(4-x_circle.^2);
y_circle_lower = -sqrt(4-x_circle.^2);
hold all
plot(x_circle,y_circle_upper,'r',x_circle,y_circle_lower,'r');

% subplot(2,4,[3,4,7,8]);
% plot(latitude,longitude)
% grid
% axis equal
% title('GPS location');
% xlabel('latitude');
% ylabel('longitude');
% hold all
% plot(latitude(1),longitude(1),'ro');
% hold all
% plot_google_map

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                       % % ACCEL LOCATION FINDER% %
                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

S_x = 0;
S_y = 0;
% for counter = 2:length(time_acc)
%     dt = (time_acc(counter)-time_acc(counter-1))^2;
%     S_x(counter) = S_x(counter-1)+.5*acc_x(counter)*dt;
%     S_y(counter) = S_y(counter-1)+.5*acc_y(counter)*dt;
% end

yaw = -215*pi/180;
pitch = 0;
roll = 0;
for counter = 2:length(time_acc)
    dt = (time_acc(counter)-time_acc(counter-1));
    dTheta = yaw_s(counter)*dt;
    yaw(counter) = yaw(counter-1)+dTheta;
    dPhi = pitch_s(counter)*dt;
    pitch(counter) = pitch(counter-1)+dPhi;
    dOmega = roll_s(counter)*dt;
    roll(counter) = roll(counter-1)+dOmega;
end
yaw = yaw.*180./pi;
yaw = mod(yaw,360);
pitch = pitch.*180./pi;
roll = roll.*180./pi;

psi = 0;
V = 0;
dPsi = 0;
for counter = 2:length(time_kistler)
    dt = (time_kistler(counter)-time_kistler(counter-1));
%     dPsi = atan(V_y(counter)/V_x(counter));
%     if(isnan(dPsi))
%         psi(counter) = psi(counter-1)+0;
%     else
%         psi(counter) = psi(counter-1)+atan(V_y(counter)/V_x(counter));
%     end
    V(counter) = sqrt(V_x(counter)^2+V_y(counter)^2);
    dS = V(counter)*dt;
    Psi = yaw(round(counter*length(time_acc)/length(time_kistler)))*pi/180;
    S_x(counter) = S_x(counter-1)+dS*cos(Psi);%*cos(psi(counter));%*cos(psi(counter));
    S_y(counter) = S_y(counter-1)-dS*sin(Psi);%*sin(psi(counter));%*sin(psi(counter));
end
V_ax = 0;
V_ay = 0;
V_az = 0;
for counter = 2:length(time_acc)
    dt = (time_acc(counter)-time_acc(counter-1));
    V_ax(counter) = V_ax(counter-1)+cos(pitch(counter)*pi/180)*acc_x(counter)*dt;
    V_ay(counter) = V_ay(counter-1)+acc_y(counter)*dt;
    V_az(counter) = V_az(counter-1)+(acc_z(counter)-g)*dt;
end

% subplot(1,2,1)
% plot(S_x,S_y);
% grid
% axis equal
% title('Accel locator');
subplot(2,4,[3,4]);
plot(time_kistler,V_x,time_acc,V_ax,'r');
grid
xlim([0, max(time_acc)]);
xlabel('t[s]');
ylabel('V_x[m/s]');
legend('Vx_{kistler}','Vx_{calculated}');
subplot(2,4,[7,8]);
plot(time_kistler,V_y,time_acc,V_ay,'r');
grid
xlim([0, max(time_acc)]);
xlabel('t[s]');
ylabel('V_y[m/s]');
legend('Vy_{kistler}','Vy_{calculated}');

f2=figure_fullScreen;
subplot(121);
plot(S_x,S_y);
grid
title('Track');
xlabel('[m]');
ylabel('[m]');
hold all
plot(0,0,'ro');
subplot(122);
plot(latitude,longitude)
grid
axis equal
title('GPS location');
xlabel('latitude');
ylabel('longitude');
hold all
plot(latitude(1),longitude(1),'ro');

f3 = figure_fullScreen;
subplot(311)
plot(time_acc,yaw)
title('Yaw');
xlabel('t[s]');
ylabel('[deg]');
grid
xlim([0 max(time_acc)]);
subplot(312)
plot(time_acc,pitch)
title('Pitch');
xlabel('t[s]');
ylabel('[deg]');
grid
xlim([0 max(time_acc)]);
subplot(313)
plot(time_acc,roll)
title('Roll');
xlabel('t[s]');
ylabel('[deg]');
grid
xlim([0 max(time_acc)]);

% S_x = 0;
% S_y = 0;
% for counter = 2:length(time_acc)
%     dt = (time_acc(counter)-time_acc(counter-1));
%     V = sqrt(V_ax(counter)^2+V_ay(counter)^2);
%     S_x(counter) = S_x(counter-1)+dt*V*cos(yaw(counter)*pi/180);
%     S_y(counter) = S_y(counter-1)+dt*V*sin(yaw(counter)*pi/180);
% end