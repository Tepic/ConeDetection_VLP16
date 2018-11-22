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
scan = 1;
closest = 1.2;
furtherest = 12;
iter = 0;
i=41000;
while i<117197
    i=i+1;
    iter = iter+1;
    velodyne_points = readMessages(velodyne_points_select,i);
    ptcloud = velodyne_points{1};
    if(iter==1)
        LIDAR = readXYZ(ptcloud);    
    else
        lidar_temp = readXYZ(ptcloud);
        LIDAR = [LIDAR; lidar_temp];
    end
    
    if mod(i,75) == 0
        tic
        hold off
        LIDAR = pointCloud(LIDAR);

        mark = [];
        for points=1:LIDAR.Count
            xx = LIDAR.Location(points,1);
            yy = LIDAR.Location(points,2);
            zz = LIDAR.Location(points,3);
            
            middle_boundry = 7;
            if(xx>0 && abs(yy)<8 && sqrt(xx^2+yy^2)>1.42 && xx<10)
                if (zz>-0.1 && sqrt(xx^2+yy^2)<middle_boundry)
                    Lidar_z = LIDAR.Location(:,3);

                    z_02 = find(Lidar_z<0.2);
                    locs = z_02(find(Lidar_z(z_02)>-.1));

                    x = LIDAR.Location(locs,1);
                    y = LIDAR.Location(locs,2);
                    z = LIDAR.Location(locs,3);

                    LIDAR_close = pointCloud([x,y,z]);
                    [indices,dists] = findNeighborsInRadius(LIDAR_close,[xx yy zz],.5);%,'MaxLeafCheck',100);

                    if(mean(dists)<.1 && length(indices)>1)
                       mark = [mark; xx yy zz]; 
                    end 
                else
                    if(zz<0.2 && sqrt(xx^2+yy^2)>middle_boundry && sqrt(xx^2+yy^2)<10)
                        [indices,dists] = findNeighborsInRadius(LIDAR,[xx yy zz],1);

                        if(mean(dists)<.1 && length(indices)<15 && length(indices)>1)
                           mark = [mark; xx yy zz]; 
                        end 
                    end
                end
            end
        end

        toc
        [r,c]=size(mark);
        if(r>0)
            scatter3(mark(:,1),mark(:,2),mark(:,3),'ro');
            hold all
        end
        scatter3(LIDAR.Location(:,1),LIDAR.Location(:,2),LIDAR.Location(:,3),1,'k.')
        hold all
        xlim([-10 10])
        ylim([-10 10])
        view(2)
        iter = 0;
        title(num2str(i));
        pause(0.01)
        i=i+2*75;
    end
end