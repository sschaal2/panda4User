sensor_names = {'R_RF_SG', 'R_LF_SG', 'R_MF_SG'};

%alpha = 48/180*pi + pi/2;
alpha = (90 - 39.56) / 180.0 * pi;
finger_length = 0.056;

%force = [0.879462 5.476593 9.988319 14.710652 19.471156 24.174851 28.867988];

%torque = force * sin(alpha) * finger_length / 3;
%torque = -1* [-0.0126563 -0.0788131 -0.143741 -0.2117 -0.280208 -0.347898 -0.415437];
torque = -1 * [-0.0111673 -0.0780262 -0.145813 -0.214238 -0.280621 -0.348842 -0.417995];


%avg{1} = [1864.75 1986.07 2043.31 2102.54 2159.6 2229.62 2287.02];
%avg{3} = [1916.32 1927.87 2000.22 2083.19 2162.73 2245.47 2323.27];
%avg{2} = [2120.58 2145.25 2159.45 2207.33 2246.69 2292.79 2330.58];

avg{1} = [1851.73 2029.43 2207.56 2376.59 2538.96 2677.35 2818.56];
avg{3} = [1891.01 1900.79 1984.14 2142.17 2296.78 2452.06 2597.14];
avg{2} = [2100.39 2201.99 2346.44 2485.17 2594.37 2704.85 2830.92];

for i = 1:3

    reg = [avg{i}', -ones(length(torque), 1)];

    res = pinv(reg) * torque';

    subplot(3,1,i);
    %plot(avg{i}, torque, 'rx', 'linewidth', 4);
    plot(torque, avg{i}, 'gx', 'linewidth', 4);
    hold on;
    %plot(avg{i}, avg{i}*res(1) - res(2), '--b', 'linewidth', 2);

    fprintf('\n sensor: %s\t', sensor_names{i}); 
    fprintf('Slope: %5f\t',res(1));
    fprintf('Offset: %5f\n',res(2)/res(1));
    
end