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

scan = 0;

% i is startin reading point (data)
i=41000; %32000;
while i<117197
    i=i+1;
    scan = scan+1;
    
    % IMPORT READING DATA
    velodyne_points = readMessages(velodyne_points_select,i);
    
    % STORE DATA
    ptcloud = velodyne_points{1};
    if(scan==1)
        LIDAR = readXYZ(ptcloud);    
    else
        lidar_temp = readXYZ(ptcloud);
        LIDAR = [LIDAR; lidar_temp];
    end
    
    % IF ONE CIRCLE SCAN HAS BEEN DONE
    % DO PROCESSING
    if mod(i,75) == 0
        
        tic
        hold off
        
        % CONVERT READ DATA
        % MATRIX to POINTCLOUD
        LIDAR = pointCloud(LIDAR);
        
        mark = [];  % COORDINATES OF CONES
        ids = [];   % indices of point that is part of cone
        
        % traverse through all LIDAR DATA
        % LIDAR.Location(:,:) = matrix N x 3
        % N = LIDAR.Cound <=> number of points (rows)        
        % points = current row
        for points=1:LIDAR.Count
            
            % current point that we examine
            xx = LIDAR.Location(points,1);
            yy = LIDAR.Location(points,2);
            zz = LIDAR.Location(points,3);
            
            % there are 2 different approaches
            % points closer than middle_boundry [in meters]
            % points further than middle_boundry [in metters]
            % y_boundry makes tunnel vision ylim[-y_boundry +y_boundry]
            middle_boundry = 7;
            y_boundry = 4;
            
            % next line is commented in order to check results if this
            % constraint is being added to the following if statement            
            % (180/pi)*atan(abs(yy)/xx)<65 && 
            
            % CONSTRAINTS:
            % 1 - points only infront (X>0)
            % 2 - y_boundry tunnel vision
            % 3 - omit points closer than 1.42m (that is vehicle itself)
            % 4 - saturate furtherest points to 10m
            if(xx>0 && abs(yy)<y_boundry && sqrt(xx^2+yy^2)>1.42 && xx<10)
                
                % IF
                %   point is closer than middle_boundry
                % ELSE
                %   point is further than middle_boundry
                % END
                if (zz>-0.1 && sqrt(xx^2+yy^2)<middle_boundry)
                    % in inner circle consider only points higher
                    % than -0.1m
                    % LIDAR is approx. posted at the height of 20cm
                              
                    Lidar_z = LIDAR.Location(:,3);
                    z_02 = find(Lidar_z<0.2);
                    locs = z_02(find(Lidar_z(z_02)>-.1));          
                    % locs are points which are -10cm < heigth < 20cm
                    % height of the cone is not bigger than 30cm

                    x = LIDAR.Location(locs,1);
                    y = LIDAR.Location(locs,2);
                    z = LIDAR.Location(locs,3);
                    LIDAR_close = pointCloud([x,y,z]);
                    
                    % find all points which are closer than 50cm from the
                    % examined point
                    [indices,dists] = findNeighborsInRadius(LIDAR_close,[xx yy zz],.5);
                    
                    % check if the point is not alone and if points are
                    % concentrated over small space
                    if(mean(dists)<.1 && length(indices)>1)                        
                        % CONE HAS BEEN FOUND in INNER CIRCLE
                        
                        % check if we already marked this cone
                        [member,ind] = ismember(ids,locs(indices));
                        if(sum(ind)==0)
                            ids = [ids points];
                            mark = [mark; mean(LIDAR.Location(locs(indices),:))]; % mark = [mark; xx yy zz];
                            % CONE HAS BEEN MARKED
                        end
                    end 
                else
                    if(zz<0.2 && sqrt(xx^2+yy^2)>middle_boundry && sqrt(xx^2+yy^2)<10)
                        % in outter circle consider only points lower
                        % than 0.2m in case that there is a downhill on the
                        % track
                        % LIDAR is approx. posted at the height of 20cm
                        
                        % find all points which are closer than 1m from the
                        % examined point
                        [indices,dists] = findNeighborsInRadius(LIDAR,[xx yy zz],1);
                        
                        % check if the point is not alone and if points are
                        % concentrated over small space
                        if(mean(dists)<.1 && length(indices)<15 && length(indices)>1)                        
                            % CONE HAS BEEN FOUND in INNER CIRCLE

                            % check if we already marked this cone
                            [member,ind] = ismember(ids,indices);
                            if(sum(ind)==0)
                                ids = [ids points];
                                mark = [mark; mean(LIDAR.Location(indices,:))]; % mark = [mark; xx yy zz];
                                % CONE HAS BEEN MARKED
                            end
                        end 
                    end
                end
            end
        end

        toc
        
        % DISPLAY DATA
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
        scan = 0;
        title(num2str(i));
        pause(0.01)
        i=i+2*75;
    end
end