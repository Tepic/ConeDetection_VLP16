clear all
close all
clc

load('bag.mat');
load('lidar.mat');
t = lidar.Time;
t = t-t(1);

clear all
close all
clc

load('bag.mat');
load('lidar.mat');
t = lidar.Time;
t = t-t(1);

velodyne_points_select = select(bag, 'Topic', '/velodyne_points');

gridStep = 0.1;

% f1 = figure(1);
% f1.Position = [127 173 1036 420];
figure_fullScreen;
N = 1;
scan = 1;
closest = 1.2;
furtherest = 12;
grid
for i=9:1:9%117197
    
    velodyne_points = readMessages(velodyne_points_select,i);
    xyz = readXYZ(velodyne_points{1});
    x = xyz(:,1);
    y = xyz(:,2);
    z = xyz(:,3);
    
    ptcloud = velodyne_points{1};
    ptCloud = pointCloud(readXYZ(ptcloud))%,'Color',uint8(255*readRGB(ptcloud)))
    [indices,dists] = findNeighborsInRadius(ptCloud,[-6.2496   12.7684   -0.2481],.5);
    
    ptCloudA = pcdownsample(ptCloud,'gridAverage',gridStep);

    subplot(121) 
    if mod(i,75) == 0
        scan = scan+1;
        hold off        
    else 
        hold on
    end
    pcshow(ptCloudA);
    view(2)
    title(num2str(i));
    grid
    hold all

    stepSize = floor(ptCloud.Count/ptCloudA.Count);
    indices = 1:stepSize:ptCloud.Count;
    ptCloudB = select(ptCloud, indices);

    subplot(122)
%     pcshow(ptCloudB);
    scatter3(ptCloudB.Location(:,1),ptCloudB.Location(:,2),ptCloudB.Location(:,3),'k*')
    hold all
    scatter3(ptCloudA.Location(:,1),ptCloudA.Location(:,2),ptCloudA.Location(:,3),'go')
    grid
    view(2)

    pause(0.1);
end
grid