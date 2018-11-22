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
M = 7;
N = 8;
scan = 1;
closest = 1.2;
furtherest = 12;
for i=M*74:1:N*74%117197
    
    velodyne_points = readMessages(velodyne_points_select,i);
    xyz = readXYZ(velodyne_points{1});
    x = xyz(:,1);
    y = xyz(:,2);
    z = xyz(:,3);
    
    ptcloud = velodyne_points{1};
    ptCloud = pointCloud(readXYZ(ptcloud));
    
    mark = [];
    for points=1:ptCloud.Count
        xx = ptCloud.Location(points,1);
        yy = ptCloud.Location(points,2);
        zz = ptCloud.Location(points,3);
        if(zz<0.5 && sqrt(xx^2+yy^2)<25)
            [indices,dists] = findNeighborsInRadius(ptCloud,[xx yy zz],.5);
            
            if(mean(dists)<.05 && length(indices)<8 && length(indices)>1)
               mark = [mark; xx yy zz]; 
            end    
        end
    end
    
%     subplot(121) 
%     if mod(i,75) == 0
%         scan = scan+1;
%         hold off        
%     else 
%         hold on
%     end
%     scatter3(x,y,z,1, 'k.');
% %     pcshow(ptCloud);
%     grid
%     view(2);
%     hold all
    
%     subplot(122)  
    if mod(i,75) == 0
        hold off
    else 
        hold on
    end
%     scatter3(ptCloudB.Location(:,1),ptCloudB.Location(:,2),ptCloudB.Location(:,3),'k*')
    [r,c]=size(mark);
    if(r>0)
        scatter3(mark(:,1),mark(:,2),mark(:,3),'ro');
        hold all
    end
    scatter3(ptCloud.Location(:,1),ptCloud.Location(:,2),ptCloud.Location(:,3),1,'k.')
    grid
    hold all
    xlim([-25 25])
    ylim([-25 25])
    view(2)
    
    pause(0.01);
end
grid


%             if(abs(mean(sum(ptCloud.Location(indices,3)))-.5*(max(ptCloud.Location(indices,3))+min(ptCloud.Location(indices,3))))>.1)
%             if(min(ptCloud.Location(indices,3))<0)  
%                 term = mean(abs(min(ptCloud.Location(indices,3)))+ptCloud.Location(indices,3));
%             else
%                 
%                 term = mean(ptCloud.Location(indices,3)-min(ptCloud.Location(indices,3)));
%             end
%             if(term>.2)
%                 if(length(indices)>30 && length(indices)<100)
%                    mark = [mark; xx yy zz]; 
%                 end    
%             end