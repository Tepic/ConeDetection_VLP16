function cones_location = processLIDAR(LIDAR,rowrow)

        
    % CONVERT READ DATA
    % MATRIX to POINTCLOUD
    %         LIDAR = pointCloud(LIDAR);

    found = 0;
    mark = zeros(30,3);  % COORDINATES OF CONES
    ids = zeros(30);   % indices of point that is part of cone

    % traverse through all LIDAR DATA
    % LIDAR.Location(:,:) = matrix N x 3
    % N = LIDAR.Cound <=> number of points (rows)        
    % points = current row
%     [rowrow columncolumn] = size(LIDAR);
    for points=1:rowrow
            % current point that we examine
            xx = LIDAR(points,1);
            yy = LIDAR(points,2);
            zz = LIDAR(points,3);

            % there are 2 different approaches
            % points closer than middle_boundry [in meters]
            % points further than middle_boundry [in metters]
            % y_boundry makes tunnel vision ylim[-y_boundry +y_boundry]
            outter_boundry = 10;
            middle_boundry = 7;
            inner_boundry = sqrt(2);
            back_boundry = -2;
            y__inner_boundry = 4;
            y__outter_boundry = 4;

            % next line is commented in order to check results if this
            % constraint is being added to the following if statement            
            % (180/pi)*atan(abs(yy)/xx)<65 && 

            % CONSTRAINTS:
            % 1 - points only infront (X>0)
            % 2 - y_boundry tunnel vision
            % 3 - omit points closer than 1.42m (that is vehicle itself)
            % 4 - saturate furtherest points to 10m
            if(xx>back_boundry && sqrt(xx^2+yy^2)>inner_boundry && xx<outter_boundry)

                % IF
                %   point is closer than middle_boundry
                % ELSE
                %   point is further than middle_boundry
                % END
                if (zz>-0.1 && sqrt(xx^2+yy^2)<middle_boundry && abs(yy)<y__inner_boundry)
                    % in inner circle consider only points higher
                    % than -0.1m
                    % LIDAR is approx. posted at the height of 20cm

                    Lidar_z = LIDAR(:,3);
                    z_02 = find(Lidar_z<0.2);
                    locs = z_02(find(Lidar_z(z_02)>-.1));          
                    % locs are points which are -10cm < heigth < 20cm
                    % height of the cone is not bigger than 30cm

%                     x = LIDAR.Location(locs,1);
%                     y = LIDAR.Location(locs,2);
%                     z = LIDAR.Location(locs,3);
%                     LIDAR_close = pointCloud([x,y,z]);

                    % find all points which are closer than 50cm from the
                    % examined point
                    [indices,dists] = findInRadius(LIDAR(locs,:),[xx yy zz],.5,points,1);

                    % check if the point is not alone and if points are
                    % concentrated over small space
                    if(indices(1)~=0 && mean(dists)<.1)                        
                        % CONE HAS BEEN FOUND in INNER CIRCLE

                        % check if we already marked this cone
                        [member,ind] = ismember(ids,locs(indices));
                        if(sum(ind)==0)
                            found = found+1;
                            ids(found) = points;                            
                            mark(found,:) = mean(LIDAR(locs(indices),:)); % mark = [mark; xx yy zz];
                            % CONE HAS BEEN MARKED
                        end
                    end 
                else
                    if(zz<0.2 && sqrt(xx^2+yy^2)>middle_boundry && sqrt(xx^2+yy^2)<outter_boundry && abs(yy)<y__outter_boundry)
                        % in outter circle consider only points lower
                        % than 0.2m in case that there is a downhill on the
                        % track
                        % LIDAR is approx. posted at the height of 20cm

                        % find all points which are closer than 1m from the
                        % examined point
                        data_set = [LIDAR(:,1),LIDAR(:,2),LIDAR(:,3)];
                        [indices,dists] = findInRadius(data_set,[xx yy zz],1,points,0);
                        % check if there is no more than 15 points and if
                        % the point is not alone
                        % mean variance of their distance must be smaller
                        % than 10cm
                        if(indices(1)~=0 && mean(dists)<.1)                        
                            % CONE HAS BEEN FOUND in INNER CIRCLE

                            % check if we already marked this cone
                            [member,ind] = ismember(ids,indices);
                            if(sum(ind)==0)
                                found = found+1;
                                ids(found) = points;
                                mark(found,:) = mean(LIDAR(indices,:)); % mark = [mark; xx yy zz];
                                % CONE HAS BEEN MARKED
                            end
                        end 
                    end
                end
            end
            if(found==30)
                break;
            end
    end
    if(found>1)
        cones_location = mark(1:found,:);
    else
        cones_location = [];
    end
end