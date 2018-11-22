clear all
close all
clc

% % script to extract some data from the .bag-file

% % load

%open file. look at the workspace variable bag to view contents like
%available topics etc.
% bag = rosbag('G:\greenteamE7001\amzdata\2017-05-10-13-14-16.public-release.bag')

% select a subset of the rosbag, in this case messages from the
% /velodyne_points topic
load('bag.mat');
lidarbag = select(bag, 'Topic', '/velodyne_points');
% % process example, plot lidar data

% with readMessages messages can be extracted from the rosbag variable.
% 
% output is always a cell array, even with one single message. Access with
% msg{i}. Data type of the extracted message depends on the ros message type.
% for i=10001:1:10002
%     
%     msg = readMessages(lidarbag, i);
%     axis square
%     
%     if mod(i,70) == 0
%         hold off
%     else 
%         hold on
%     end
%     scatter3(msg{1});
%     set(gca,'XLim',[-12 12],'YLim',[-12 12],'ZLim',[-12 12])
%     view(2);
%     
%     pause(0.5)
% end

    
velodyne_points_select = select(bag, 'Topic', '/velodyne_points');
figure
f1.Position = [210 41 784 632];
i=39060;
while i<117197    
%     axis square
    velodyne_points = readMessages(velodyne_points_select,i);
    xyz = readXYZ(velodyne_points{1});
    
    x = xyz(:,1);
    y = xyz(:,2);
    z = xyz(:,3);
     
    if mod(i-25,75) == 0
        i = i+2*75+1;
        hold off        
    else 
        i=i+1;
        f1 = figure(1);
        f1.Position = [210 41 784 632];
        xlim([-15 20]);
        ylim([-50 30]);
        axis square
        hold on
    end
%     pcshow(xyz);
    scatter3(x,y,z,1, 'k.');
    title(num2str(i));
% %     set(gca,'XLim',[-12 12],'YLim',[-12 12],'ZLim',[-12 12])
% %     grid
    view(2);
%     view([0,45]);
    pause(0.01);
end