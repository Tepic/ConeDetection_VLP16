clear all
close all
clc

load('bag.mat');
load('lidar.mat');
t = lidar.Time;
t = t-t(1);

velodyne_points_select = select(bag, 'Topic', '/velodyne_points');

f1 = figure(1);
% f1.Position = [127 173 1036 420];
f1.Position = [291 47 766 632];
% figure_fullScreen;
M = 1;
N = 10;

scan = 186;
iter = 0;

% i is startin reading point (data)
i=40951; %32000;
iter = 1;
XYZ = [];
while i<73650 %117197
    i=i+1;
    
    % IMPORT READING DATA
    velodyne_points = readMessages(velodyne_points_select,i);
    
    % STORE DATA
    ptcloud = velodyne_points{1};
    if(iter==1)
        LIDAR = readXYZ(ptcloud);
        [r c] = size(LIDAR);
        time = velodyne_points{1,1}.Header.Stamp.Sec+velodyne_points{1,1}.Header.Stamp.Nsec;
        XYZ = [XYZ; LIDAR,ones(r,1).*r,ones(r,1).*time];
    else
        if(length(ptcloud.Data)~=0)
            lidar_temp = readXYZ(ptcloud);
            LIDAR = [LIDAR; lidar_temp];
            [r c] = size(LIDAR);
            time = velodyne_points.Header.Stamp.Sec+velodyne_points{1,1}.Header.Stamp.Nsec;
            XYZ = [XYZ; LIDAR, r, ones(r,1).*time];
            iter = 0;
        end
    end
end
save('XYZ.mat','LIDAR')