function [index, distance] = findInRadius(data_set,point,R,location,boundry)

    index = [];
    distance = [];
    found = 0;
    [L c] = size(data_set);
%     location-ceil(length/4):location+ceil(length/4)
    if(boundry==0)
        boundry=15;
    else
        boundry=50;
    end
    for row = 1:L
%         if(row~=location)
%             r = mod(row,length);
%             if(r==0)
%                 r=length;
%             end
%             if(r==14462)
%                 disp([num2str(row),' ', num2str(location),' ',num2str(length)]);
%             end
            r = row;
            d = sqrt((data_set(r,1)-point(1))^2+(data_set(r,2)-point(2))^2);
            if(d<R && d~=0)
                found = found+1;
                if(found>boundry)
                    index = 0;
                    distance = [];
                    break;
                end                
                index = [index r];
                distance = [distance d];
            end
%         end            
    end
    if(length(index)<2)
        index = 0;
        distance = [];
    end
end