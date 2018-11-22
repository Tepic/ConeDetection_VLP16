clear all
close all
clc

load('bag.mat');
load('lidar.mat');
t = lidar.Time;
t = t-t(1);

velodyne_points_select = select(bag, 'Topic', '/velodyne_points');

gridStep = 0.1;
ptCloudA = pcdownsample(ptCloud,'gridAverage',gridStep);

% f1 = figure(1);
% f1.Position = [127 173 1036 420];
figure_fullScreen;
N = 4;
scan = 1;
closest = 1.2;
furtherest = 12;
for i=1:1:N*74%117197
    
    tic
    velodyne_points = readMessages(velodyne_points_select,i);
    xyz = readXYZ(velodyne_points{1});
    x = xyz(:,1);
    y = xyz(:,2);
    z = xyz(:,3);
        
    subplot(121)  
    if mod(i,75) == 0
        scan = scan+1;
        hold off        
    else 
        hold on
    end
    scatter3(x,y,z,1, 'k.');%,'LineWidth',1.1);
    set(gca,'XLim',[-12 12],'YLim',[-12 12],'ZLim',[-12 12])
    title(['Scan No.',num2str(scan)]);
    view(2);
    hold all
    
    % detect edges higher than 30cm
    lower = z>=min(z);
    upper = z<.3;
    position = find(lower==upper);
           
    fields = []; % delete[]
    density = 0.4; % [m]
    
    alone_fields = [0 0];
    for element = 1:length(x)
        % check edges only BELOW 30cm        
        if(isempty(find(position==element))==0)            
            x_e = x(element);
            x_l = floor(x_e/density);
            x_u = ceil(x_e/density);

            y_e = y(element);
            y_l = floor(y_e/density);
            y_u = ceil(y_e/density);

            row = 0.4*y_l+.2;
            column = 0.4*x_l+.2;
            fields = [fields; [column row]];

            % avoid fields to be marked twice                      
            locus_x = find(alone_fields(:,1)==single(column));
            locus_y = find(alone_fields(:,2)==single(row));

            if(isempty(locus_x) || isempty(locus_y))
                % if this x or y coordinate has not been marked yet - note
                % this point
                alone_fields = [alone_fields; [column row]];
            else
                % if both coordinates already exast in the dataset
                % then check if they are for the same point or originate
                % from two different points
                flag = 0;
                for l_x = 1:length(locus_x)
                    for l_y = 1:length(locus_y)
                        if(locus_x(l_x)==locus_y(l_y))
                            flag = 1;
                            break;
                        end
                    end
                    if(flag==1)
                        break;
                    end
                end

                % flag will stay 0 if found coordinates belongs to two
                % different coordinates
                % therefor add this point to the dataset alone_fields
                if(flag==0)                        
                    alone_fields = [alone_fields; [column row]];
                end
            end
        end %if(isempty(find(position==element))==0)
        
    end
    
    % mark points which does not have any other in the neighborhood
    % in diameter of 1.5*0.4m*sqrt(2) => D~0.88m
    % which means that there are no points next to the one algorithm checks
    mark = [];
    alert = 0;
    alone_fields = alone_fields(2:end,:);
    [r c] = size(alone_fields);
    d = [];
    nearby = 0;
    for r_c_outer = 1:r
        for r_c_inner = 1:r-1
            distance = sqrt((alone_fields(r_c_outer,1))^2+(alone_fields(r_c_outer,2))^2);            
            if(distance>closest && distance<furtherest)            
                r_c_temp = mod(r_c_inner+r_c_outer,r);
                if(r_c_temp==0)
                    r_c_temp=r;
                end
                d = [d sqrt(sum((alone_fields(r_c_outer,:)-alone_fields(r_c_temp,:)).^2))];
                if(d(end)<2*2.2*density)
                    if(nearby==0)
                        nearby=1;
                    else
                        nearby=0;
                        alert = 1;                   
                        break;
                    end
                end
            else
                alert = 1;
            end
        end
        if(alert==0)
            mark = [mark; alone_fields(r_c_outer,:)];
        else
            alert = 0;
        end
    end
    
    subplot(122)  
    if mod(i,75) == 0
        hold off
    else 
        hold on
    end

%     hold off
    scatter3(x(position),y(position),z(position),1,'k')
    set(gca,'XLim',[-12 12],'YLim',[-12 12],'ZLim',[-12 12])
    hold all
    
    [r,c]=size(fields);
    scatter3(fields(:,1),fields(:,2),ones(r,1),'ro');
    hold all
    
    [r,c]=size(alone_fields);
    scatter3(alone_fields(:,1),alone_fields(:,2),ones(r,1),'go');
    hold all
    
    [r,c]=size(mark);
    if(r>0)
        scatter3(mark(:,1),mark(:,2),ones(r,1),'b*');
        hold all
    end
    view(2)
    
    dt = toc;
%     wait = t(i)-t(i-1)-dt;
    pause(0.01);
end

% xticks([-12:0.4:12])
% yticks([-12:0.4:12])
grid