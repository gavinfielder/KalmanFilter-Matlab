%% Loads Data into workspace
% 

if exist(filename)
    data = load(filename,'-ascii');
    accel_x = data(1:1500);
    accel_y = data(1501:3000);
    accel_z = data(3001:4500);
    gyro_y = data(4501:6000);
    gyro_z = data(6001:7500);
    encoder = data(7501:9000);
    velocity = data(9001:10500);
end
clear data;
