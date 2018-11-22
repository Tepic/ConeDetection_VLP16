clear all
close all
clc
load('bag.mat');
% clearvars -except bag

start_time = bag.StartTime;

imu_select                  = select(bag, 'Topic', '/imu');
velodyne_points_select      = select(bag, 'Topic', '/velodyne_points');
optical_speed_sensor_select = select(bag, 'Topic', '/optical_speed_sensor');
wheel_rpm_select            = select(bag, 'Topic', '/wheel_rpm');
gps_select                  = select(bag, 'Topic', '/gps');

imu                  = readMessages(imu_select,1);
velodyne_points      = readMessages(velodyne_points_select,1);
optical_speed_sensor = readMessages(optical_speed_sensor_select,1);
wheel_rpm            = readMessages(wheel_rpm_select,1);
gps                  = readMessages(gps_select,1);

load('lidar.mat');
load('accel.mat');
load('kistler.mat');
load('wheels.mat');
load('gps_data.mat');
% lidar = timeseries(select(bag, 'Topic', '/velodyne_points'));
% accel = timeseries(select(bag, 'Topic', '/imu'));
% kistler = timeseries(select(bag, 'Topic', '/optical_speed_sensor'));
% wheels = timeseries(select(bag, 'Topic', '/wheel_rpm'));
% gps_data = timeseries(select(bag, 'Topic', '/gps'));


disp('Done!');

% imu = timeseries(select(bag,'Topic', '/imu'),'LinearAcceleration.X','LinearAcceleration.Y','LinearAcceleration.Z');
% ang = timeseries(select(bag,'Topic', '/imu'),'AngularVelocity.X','AngularVelocity.Y','AngularVelocity.Z');
% 
% acc_time = imu.Time-start_time;
% acc_X = imu.Data(:,1);
% acc_Y = imu.Data(:,2);
% acc_Z = imu.Data(:,3);
% 
% figure
% plot(acc_time,imu.Data);
% 
% disp('Done');