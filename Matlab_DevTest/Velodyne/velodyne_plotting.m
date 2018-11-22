clear all
close all
clc

load('bag.mat');
velodyne_points_select = select(bag, 'Topic', '/velodyne_points');

f1 = figure(1);
f1.Position = [127 173 1036 420];
% figure_fullScreen;
for i=1:1:74%117197
    
    axis square
    
%     if mod(i,75) == 0
%         if(i==150)
%             break;
%         end
%         hold off
%     else 
%         hold on
%     end
    
    velodyne_points = readMessages(velodyne_points_select,i);
    xyz = readXYZ(velodyne_points{1});
    x = xyz(:,1);
    y = xyz(:,2);
    z = xyz(:,3);
    
%     x = 0.*x(x>12);
%     x = 0.*x(x<-12);    
%     y = 0.*y(y>12);
%     y = 0.*y(y<-12);

%     z = z.*(z>0.1).*(z<0.3);    
%     x = x.*(z~=0);
%     y = y.*(y~=0);

    subplot(121)  
    if mod(i,75) == 0
        hold off        
    else 
        hold on
    end
    scatter3(x,y,z,1, 'k.');%,'LineWidth',1.1);
    set(gca,'XLim',[-12 12],'YLim',[-12 12],'ZLim',[-12 12])
%     grid
    view(2);%[0,45]);y
    
    lower = z>min(z);
    upper = z<.3;
    position = find(lower==upper);
    
    subplot(122)  
    if mod(i,75) == 0
        hold off
    else 
        hold on
    end
    
    delete = [];
    density = 0.4; % [m]
    for row=-29:30
        for column=-29:30
            loc_upper = find(x<=row*density);
            loc_lower = find(x>(row-1)*density);
            if(length(loc_upper)>0 && length(loc_lower)>0)                
                
                loc = [];
                if(length(loc_upper)<length(loc_lower))
                    for kk=1:length(loc_upper)
                        loc = [loc find(loc_lower==loc_upper(kk))];
                    end
                else                    
                    for kk=1:length(loc_lower)
                        loc = [loc find(loc_upper==loc_lower(kk))];
                    end
                end           
                
                if(length(loc)~=0)
                    bigger = find(y(loc)<=column*density);
                    smaller = find(y(loc)>(column-1)*density);
                    
                    ys = [];
                    if(length(bigger)<length(smaller))
                        for kk=1:length(bigger)
                            ys = [ys find(smaller==bigger(kk))];
                        end
                    else                    
                        for kk=1:length(smaller)
                            ys = [ys find(bigger==smaller(kk))];
                        end
                    end  
                
                    if(length(ys)~=0)
                        delete = [delete ys];
                    end
                end
            end
        end
    end
    x(delete)=-999;
    y(delete)=-999;
    
    scatter3(x(position),y(position),z(position),1,'k')
    set(gca,'XLim',[-12 12],'YLim',[-12 12],'ZLim',[-12 12])
%     grid
    view(2)
    
%     subplot(2,2,[3 4])  
%     if mod(i,75) == 0
%         if(i==150)
%             break;
%         end
%         hold off
%     else 
%         hold on
%     end
%     plot(z,'b');
%     hold on
%     plot(z.*(z>0.1).*(z<0.3),'g');
%     grid
%     xlim([0 length(z)]);
    
    pause(0.01)
end