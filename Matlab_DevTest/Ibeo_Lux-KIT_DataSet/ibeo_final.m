clear all
close all
clc

load('ibeo_LIDAR_L_data.mat');
load('ibeo_LIDAR_R_data.mat');

% t = lidar.Time;
% t = t-t(1);

f1 = figure(1);
f1.Position = [291 47 766 632];
% figure_fullScreen;

scan = 1;
iter = 0;
total_time = 0;
time_L = LIDAR_left{1}.Time;
num_messages = length(LIDAR_left);
for i=2:num_messages
    iter = iter+1;
    
    % IMPORT READING DATA
    time_L = [time_L; LIDAR_left{i}.Time];
    LIDAR_L = [LIDAR_left{i}.X LIDAR_left{i}.Y LIDAR_left{i}.Z];
    LIDAR_R = [LIDAR_right{i}.X LIDAR_right{i}.Y LIDAR_right{i}.Z];
    
    LIDAR_L(isnan(LIDAR_L)) = [];
    LIDAR_R(isnan(LIDAR_R)) = [];
    
    if(isempty(LIDAR_L)~=1 && isempty(LIDAR_R)~=1)
        LIDAR_L = [sqrt((LIDAR_L(:,1)).^2+(LIDAR_L(:,2)).^2).*cos(-pi/6+atan(LIDAR_L(:,2)./LIDAR_L(:,1))),...
                   -0.31+sqrt((LIDAR_L(:,1)).^2+(LIDAR_L(:,2)).^2).*sin(-pi/6+atan(LIDAR_L(:,2)./LIDAR_L(:,1))),...
                   LIDAR_L(:,3)];

        LIDAR_R = [sqrt((LIDAR_R(:,1)).^2+(LIDAR_R(:,2)).^2).*cos(pi/6+atan(LIDAR_R(:,2)./LIDAR_R(:,1))),...
                   0.31+sqrt((LIDAR_R(:,1)).^2+(LIDAR_R(:,2)).^2).*sin(pi/6+atan(LIDAR_R(:,2)./LIDAR_R(:,1))),...
                   LIDAR_R(:,3)];
        LIDAR = [LIDAR_L; LIDAR_R];
    
        % IF ONE CIRCLE SCAN HAS BEEN DONE
        % DO PROCESSING
        tic 
%             [rowrow_L columncolumn_L] = size(LIDAR_L);        
%             mark_L = processLIDAR(LIDAR_L,rowrow_L); 
%             
%             [rowrow_R columncolumn_R] = size(LIDAR_R);        
%             mark_R = processLIDAR(LIDAR_R,rowrow_R); 
            
            [rowrow columncolumn] = size(LIDAR);        
            mark = processLIDAR(LIDAR,rowrow); 
        t = toc;

            total_time = total_time+t;
            disp(['Scanned point: ', num2str(rowrow)]);
            disp(['Average time:  ', num2str(t/rowrow),'s']);
            disp(['Elapsed time:  ', num2str(t),'s']);
            disp(['ETA 2scans:  ', num2str(1e-4*(time_L(i)-time_L(i-1))),'s']);
            disp(['Average scan time:  ', num2str(total_time/scan),'s']);
            disp('__________________________________');

            % DISPLAY DATA
            figure(f1.Number)   ;     
%             [r,c]=size(mark_L);
%             if(r>0)
%                 scatter3(mark_L(:,1),mark_L(:,2),mark_L(:,3),'ro');
%                 hold all
%             end
%             scatter3(LIDAR_L(:,1),LIDAR_L(:,2),LIDAR_L(:,3),1,'k.')
%             
%             [r,c]=size(mark_R);
%             if(r>0)
%                 scatter3(mark_R(:,1),mark_R(:,2),mark_R(:,3),'bo');
%                 hold all
%             end
%             scatter3(LIDAR_R(:,1),LIDAR_R(:,2),LIDAR_R(:,3),1,'k.')
            
            [r,c]=size(mark);
            if(r>0)
                scatter3(mark(:,1),mark(:,2),mark(:,3),'ro');
                hold all
            end
            scatter3(LIDAR(:,1),LIDAR(:,2),LIDAR(:,3),1,'k.')
            hold off
            xlim([-1 20])
            ylim([-10 10])
            view(2)
            iter = 0;
            title(['ibeo: ' num2str(i)]); 

%             saveas(f1,[pwd '/Scans/Scan ',num2str(scan),'.png']);
            scan = scan+1;
            pause(0.01)       
    end
end