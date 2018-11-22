clear all
close all
clc

filename = '2017-05-10-13-14-16.public-release.bag';
bag = rosbag(filename);
disp('Imported');

% bagselect1 = select(bag, 'Topic', '/odom')
% bagselect2 = select(bag, 'Time', [start start + 30], 'Topic', '/odom')
% bagselect3 = select(bagselect2, 'Time', [205 206])
% selectOptions = {'Time', [start, start+1; start+5, start+6], 'MessageType', {'sensor_msgs/LaserScan', 'nav_msgs/Odometry'}};
% bagselect4 = select(bag, selectOptions{:})
% msgs = readMessages(bagselect3, [1 2 3 7])
% msgs{2}
% ts = timeseries(bagselect3, 'Pose.Pose.Position.X', 'Twist.Twist.Angular.Z')
% ts.Data