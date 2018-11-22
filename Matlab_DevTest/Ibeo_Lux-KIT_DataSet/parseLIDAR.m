function [LIDAR_left LIDAR_right] = parseLIDAR()
    filename_left_lidar = "raw_points_left.txt";
    filename_right_lidar = 'raw_points_right.txt'; 
    
    LIDAR_left_txt = importdata(filename_left_lidar);
    LIDAR_right_txt = importdata(filename_right_lidar);

    time = [];
    x = []; y = []; z = [];
    LIDAR = [];
    depth = [];
    for scan = 1:length(LIDAR_left_txt)
        data=[];
        semicolumn = 0;
        comma = 0;
        
        text_Data = LIDAR_left_txt{scan};
        iter = 1;
        while iter <= length(text_Data)
            
            % EXTRACT TIME
            while(semicolumn==0)
                if(text_Data(iter)==';')
                    time = [time; str2num(data)];
                    data = [];
                    semicolumn = 1;
                    iter = iter+1;
                else                    
                    data = [data text_Data(iter)];
                end
                iter = iter+1; 
            end           
            if(iter > length(text_Data))
                break;
            end
            
            if(text_Data(iter)==',')
                if(comma==0)
                    x = [x; str2num(data)];
                else
                    y = [y; str2num(data)];
                end
                data = [];
                comma = 1;
            else
                if(text_Data(iter)==';')
                    z = [z; str2num(data)];
                    data = [];
                    comma = 0;
                else
                    data = [data text_Data(iter)];
                end
            end
            iter = iter+1;
        end
        LIDAR_left{scan}.Time = time;
        LIDAR_left{scan}.X = x;
        LIDAR_left{scan}.Y = y;
        LIDAR_left{scan}.Z = z;
    end
    
    time = [];
    x = []; y = []; z = [];
    LIDAR = [];
    depth = [];
    for scan = 1:length(LIDAR_right_txt)
        data=[];
        semicolumn = 0;
        comma = 0;
        
        text_Data = LIDAR_right_txt{scan};
        iter = 1;
        while iter <= length(text_Data)
            
            % EXTRACT TIME
            while(semicolumn==0)
                if(text_Data(iter)==';')
                    time = [time; str2num(data)];
                    data = [];
                    semicolumn = 1;
                    iter = iter+1;
                else                    
                    data = [data text_Data(iter)];
                end
                iter = iter+1;
            end           
            if(iter <= length(text_Data))
                break;
            end
            
            if(text_Data(iter)==',')
                if(comma==0)
                    x = [x; str2num(data)];
                else
                    y = [y; str2num(data)];
                end
                data = [];
                comma = 1;
            else
                if(text_Data(iter)==';')
                    z = [z; str2num(data)];
                    data = [];
                    comma = 0;
                else
                    data = [data text_Data(iter)];
                end
            end
            iter = iter+1;
        end
        LIDAR_righ{scan}.Time = time;
        LIDAR_righ{scan}.X = x;
        LIDAR_righ{scan}.Y = y;
        LIDAR_righ{scan}.Z = z;
    end
end