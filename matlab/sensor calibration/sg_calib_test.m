function sg_calib_test

%%assume we have in a file the force variable applied to the finger tip
%%during the time stamps time_stamp, this file makes a linear regression to
%%compute the slope and offset of the Strain gages
% we assume here the initial slope = 1 and offset = 0 in the
% ExtraSensors.cf file

%clear all; close all;

sensor_names = {'R_RF_SG', 'R_LF_SG', 'R_MF_SG'};
file = {'d00526', 'd00525', 'd00527'};
orig_offset = [1820.72 2302.61 1914.59];
orig_slope = [-0.000380467 -0.000441207 -0.000381114];

time_stamp{1} = [0 3; 5 9; 12 15; 18 21; 23 27; 30 32; 35 39; 42 45; 49 52; 55 57];

time_stamp{2} = [1 4; 7 10; 13 16; 19 22; 24 27; 29 32; 36 38; 41 43; 45 47.5; 50 54];

time_stamp{3} = [1 5; 6 9; 10 13; 14 19; 22 25; 28 31; 36 39; 43 46; 49 53; 56 59.9];


ounce = 0.0283495231;
force = [0 2 4 6 8 10 13 16 20 24] * 9.81 * ounce;

%alpha = 48/180*pi + pi/2;
alpha = (90 - 39.56) / 180.0 * pi;

finger_length = 0.056;

torque = force * sin(alpha) * finger_length;


for i = 1:3

    avg(:,i) = gather_data(file{i}, sensor_names{i}, time_stamp{i}, orig_offset(i), orig_slope(i));
    
    reg = [avg(:,i), -ones(length(force), 1)];

    res(i,:) = pinv(reg) * torque';

    subplot(3,1,i);

    %plot(avg(:,i), torque, 'kx', 'linewidth', 4);
    plot(torque, avg(:,i), 'kx', 'linewidth', 4);
    hold on;
    %plot(avg(:,i), avg(:,i)*res(i,1) - res(i,2), 'b', 'linewidth', 2);

    fprintf('\n sensor: %s\t', sensor_names{i}); 
    fprintf('Slope: %5f\t',res(i,1));
    fprintf('Offset: %5f\n',res(i,2)/res(i,1));
end


function avg = gather_data(file, sensor_name, time_stamp, orig_offset, orig_slope)
    
    [D vars freq] = clmcplot_convert(file);
    names =  {vars(:).name};
    
    t = D(:,find(strcmp(names, 'time')));
    
    sg = D(:,find(strcmp(names, sensor_name)))/orig_slope + orig_offset;
    
    limit = intersect(find(t < time_stamp(length(time_stamp(:,1)),2) + 1), find(t > time_stamp(length(time_stamp(:,1)),2) ));
    limit = limit(1);
    
    for i = 1:length(time_stamp(:,1))
        
        start = find(t(1:limit)>time_stamp(i,1));
        start = start(1);
        stop = find(t(1:limit)<time_stamp(i,2));
        stop = stop(end);
        avg(i) = mean(sg(start:stop));
        
    end
