clear all
close all
clc

load('bag.mat');
load('lidar.mat');
t = lidar.Time;
t = t-t(1);

velodyne_points_select = select(bag, 'Topic', '/velodyne_points');

f1 = figure(1);
f2 = figure(2);
% f1.Position = [127 173 1036 420];
f1.Position = [291 47 766 632];
f2.Position = [1367 45 1920 963];
% figure_fullScreen;
M = 1;
N = 10;

scan = 1;
iter = 0;
total_time = 0;

Intensity = [];
% i is startin reading point (data)
i=40951; %32000;
while i<73650 %117197
    i=i+1;
    iter = iter+1;
    
    % IMPORT READING DATA
    velodyne_points = readMessages(velodyne_points_select,i);
    
    % STORE DATA
    ptcloud = velodyne_points{1};

    data = ptcloud.Data;
    
    loc=0;
    [data_rows data_columns] = size(data);
    while(loc+20<data_rows)
        Intensity = [Intensity; typecast(uint8(data(loc+17:loc+20)'),'single')];
        loc = loc+32;
    end
    
    if(iter==1)
        LIDAR = readXYZ(ptcloud);    
    else
        if(length(ptcloud.Data)~=0)
            lidar_temp = readXYZ(ptcloud);
            LIDAR = [LIDAR; lidar_temp];
        end
    end
    
    % IF ONE CIRCLE SCAN HAS BEEN DONE
    % DO PROCESSING
    if mod(i,75) == 0
%         hold off
        
        tic 
        [rowrow columncolumn] = size(LIDAR);        
        mark = processLIDAR_mex(LIDAR,rowrow); 
        t = toc;
                      
%         b_10 = find(Intensity<10);
%         b_20 = find(Intensity<20);
%         temp = find(Intensity(b_20)>10);
%         b_20 = b_20(temp);
%         b_30 = find(Intensity<30);
%         temp = find(Intensity(b_30)>20);
%         b_30 = b_30(temp);
%         b_40 = find(Intensity<40);
%         temp = find(Intensity(b_40)>30);
%         b_40 = b_40(temp);
%         b_50 = find(Intensity<50);
%         temp = find(Intensity(b_50)>40);
%         b_50 = b_50(temp);
%         b_60 = find(Intensity<60);
%         temp = find(Intensity(b_60)>50);
%         b_60 = b_60(temp);
%         b_60_ = find(Intensity>60);
%         
%         mark_10 = processLIDAR_(LIDAR(b_10,:),length(b_10));
%         mark_20 = processLIDAR_(LIDAR(b_20,:),length(b_20));
%         mark_30 = processLIDAR_(LIDAR(b_30,:),length(b_30));
%         mark_40 = processLIDAR_(LIDAR(b_40,:),length(b_40));
%         mark_50 = processLIDAR_(LIDAR(b_50,:),length(b_50));
%         mark_60 = processLIDAR_(LIDAR(b_60,:),length(b_60));
%         mark_60_ = processLIDAR_(LIDAR(b_60_,:),length(b_60_));
        
        total_time = total_time+t;
        disp(['Scanned point: ', num2str(rowrow)]);
        disp(['Average time:  ', num2str(t/rowrow),'s']);
        disp(['Elapsed time:  ', num2str(t),'s']);
        disp(['Average scan time:  ', num2str(total_time/scan),'s']);
        disp('__________________________________');
        
        % DISPLAY DATA
        figure(f1.Number)   ;     
        [r,c]=size(mark);
        if(r>0)
            scatter3(mark(:,1),mark(:,2),mark(:,3),'ro');
            hold all
        end
        scatter3(LIDAR(:,1),LIDAR(:,2),LIDAR(:,3),1,'k.')
        hold off
%         hold all
        xlim([-10 10])
        ylim([-10 10])
        view(2)
        iter = 0;
        title(num2str(i)); 
        
%         figure(f2.Number)
%         
%         subplot(2,4,8);
%         [r,c]=size(mark);
%         if(r>0)
%             scatter3(mark(:,1),mark(:,2),mark(:,3),'ro');
%             hold all
%         end
%         scatter3(LIDAR(:,1),LIDAR(:,2),LIDAR(:,3),1,'k.')
%         hold off
% %         hold all
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         iter = 0;
%         title(num2str(i));        
%         
% %         Lidar_z = LIDAR(:,3);
% %         locs = find(Lidar_z<0.2);
% %         Intensity = Intensity(locs);
%         
% %         b_10 = find(Intensity<10);
% %         b_20 = find(Intensity<20);
% %         temp = find(Intensity(b_20)>10);
% %         b_20 = b_20(temp);
% %         b_30 = find(Intensity<30);
% %         temp = find(Intensity(b_30)>20);
% %         b_30 = b_30(temp);
% %         b_40 = find(Intensity<40);
% %         temp = find(Intensity(b_40)>30);
% %         b_40 = b_40(temp);
% %         b_50 = find(Intensity<50);
% %         temp = find(Intensity(b_50)>40);
% %         b_50 = b_50(temp);
% %         b_60 = find(Intensity<60);
% %         temp = find(Intensity(b_60)>50);
% %         b_60 = b_60(temp);
% %         b_60_ = find(Intensity>60);
% 
%         subplot(2,4,1);
%         [r,c]=size(mark_10);
% %         if(r>0)
% %             scatter3(mark_10(:,1),mark_10(:,2),mark_10(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_10,1),LIDAR(b_10,2),LIDAR(b_10,3),'k.'); 
%         title('i<10');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
%         
%         subplot(2,4,2);
%         [r,c]=size(mark_20);
% %         if(r>0)
% %             scatter3(mark_20(:,1),mark_20(:,2),mark_20(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_20,1),LIDAR(b_20,2),LIDAR(b_20,3),'g.');
%         title('10<i<20');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
%         
%         subplot(2,4,3);
%         [r,c]=size(mark_30);
% %         if(r>0)
% %             scatter3(mark_30(:,1),mark_30(:,2),mark_30(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_30,1),LIDAR(b_30,2),LIDAR(b_30,3),'m.');
%         title('20<i<30');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
%         
%         subplot(2,4,4);
%         [r,c]=size(mark_40);
% %         if(r>0)
% %             scatter3(mark_40(:,1),mark_40(:,2),mark_40(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_40,1),LIDAR(b_40,2),LIDAR(b_40,3),'r.');
%         title('30<i<40');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
%         
%         subplot(2,4,5);
%         [r,c]=size(mark_50);
% %         if(r>0)
% %             scatter3(mark_50(:,1),mark_50(:,2),mark_50(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_50,1),LIDAR(b_50,2),LIDAR(b_50,3),'b.');
%         title('40<i<50');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
%         
%         subplot(2,4,6);
%         [r,c]=size(mark_60);
% %         if(r>0)
% %             scatter3(mark_60(:,1),mark_60(:,2),mark_60(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_60,1),LIDAR(b_60,2),LIDAR(b_60,3),'c.');
%         title('50<i<60');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
%         
%         subplot(2,4,7);
%         [r,c]=size(mark_60_);
% %         if(r>0)
% %             scatter3(mark_60_(:,1),mark_60_(:,2),mark_60_(:,3),'ro');
% %             hold all
% %         end
%         scatter3(LIDAR(b_60_,1),LIDAR(b_60_,2),LIDAR(b_60_,3),'k.');
%         title('i>60');
%         xlim([-10 10])
%         ylim([-10 10])
%         view(2)
%         hold off
% %         legend('i<10','10<i<20','20<i<30','30<i<40','40<i<50','50<i<60','i>60');
% %         xlim([-10 10])
% %         ylim([-10 10])
% %         view(2)
% %         
%         saveas(f1,[pwd '/Scans/Scan ',num2str(scan),'.png']);
%         saveas(f2,[pwd '/Scans/IntensityScan ',num2str(scan),'.png']);
        scan = scan+1;
        pause(0.01)
%         i=i+75;
        Intensity = [];       
    end
end