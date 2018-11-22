function load_lux_lidar()
%LOAD_LUX_LIDAR Summary of this function goes here
%   Detailed explanation goes here
file_name_left_lidar = "raw_points_left.txt";
file_name_right_lidar = "raw_points_right.txt";

delimiterIn = ' ';
headerlinesIn = 1;

    file_data_left_lidar = fopen(file_name_left_lidar);
    file_data_right_lidar = fopen(file_name_right_lidar);
    
    data_left_lidar = fgetl(file_data_left_lidar);
    data_right_lidar = fgetl(file_data_right_lidar);
    while ischar(data_left_lidar)
        %disp(data_right_lidar)
        data_right_lidar = fgetl(file_data_right_lidar);
        data_right_lidar = strsplit(data_right_lidar,';');
        tt = data_right_lidar(2:length(data_right_lidar));
        temp_right_lidar_data_points = split(tt,",")
        right_lidar = {'timestamp',data_right_lidar(1), 'data', data_right_lidar(2:length(data_right_lidar))}
    end
%    data_left_lidar = strsplit(data_left_lidar,';')

end

