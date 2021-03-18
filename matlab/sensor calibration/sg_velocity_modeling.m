function sg_velocity_modeling

filter_order = 2;
cutoff = 0.05;
[b, a] = butter(filter_order, cutoff);

sg_names = {'R_RF_SG', 'R_MF_SG', 'R_LF_SG'};
vel_names = {'R_RF_thd', 'R_MF_thd', 'R_LF_thd'};
pos_names = {'R_RF_th', 'R_MF_th', 'R_LF_th'};
time_name = {'time'};

[D vars freq] = clmcplot_convert('d00633');
sg = clmcplot_getvariables(D, vars, sg_names);
thd = clmcplot_getvariables(D, vars, vel_names);
th = clmcplot_getvariables(D, vars, pos_names);
t = clmcplot_getvariables(D, vars, time_name);

tan_slope = 100;

grid = 0:0.5:2.0;
grid_offset = zeros(length(grid), 3);
grid_offset(:, 2) = [-0.04 -0.09 -0.06 -0.12 -0.12]';

for i = 1:3
    th(:,i) = filtfilt(b, a, th(:,i));
    off(:,i) = remove_position_offset(th(:,i), grid_offset(:,i), grid)
    sg(:,i) = filtfilt(b, a, sg(:,i));
    thd(:,i) = filtfilt(b, a, thd(:,i));
    thd_fun{i} = [tanh(tan_slope * thd(:,i)) ones(length(thd(:,i)),1)];
end

for i = 1:3
    param(:,i) = pinv(thd_fun{i}) * (sg(:,i) - off(:,i))
end

param

%%remove the position dependent offset
% grid = 0:0.1:1.94;
% grid_offset = zeros(length(grid), 3);
% for j = 1:3
%     for i = 1:length(grid)-1
%         ind = find(th(:,j) > grid(i) & th(:,j) < grid(i+1));
%         grid_offset(i,j) = median(sg(ind, j) - thd_fun{j}(ind, :) * param(:,j));
%     end
% end

figure(1)
for i = 1:3
    subplot(3,1,i);
    
%     off = remove_position_offset(th(:,i), grid_offset(:,i), grid);
   
    plot(t, sg(:,i), t, sg(:,i) - thd_fun{i} * param(:,i), ...
        t, sg(:,i) - param(2,i), ...
        t, sg(:,i) - thd_fun{i} * param(:,i) - off(:,i),...
        t, sg(:,i) - off(:,i), ...
        'linewidth', 2);
    line([t(1), t(end)], [0 0], 'linewidth', 2, 'color', 'k');
    
    [mean(sqrt( (sg(:,i)).^2 )) ...
        mean(sqrt( (sg(:,i) - thd_fun{i} * param(:,i)) .^2 )) ...
        mean(sqrt( (sg(:,i) - param(2,i)).^2 )) ...
        mean(sqrt( (sg(:,i) - thd_fun{i} * param(:,i) - off(:,i)).^2 ))]
end


%%%test
% 
% [D vars freq] = clmcplot_convert('d00632');
% sg = clmcplot_getvariables(D, vars, sg_names);
% thd = clmcplot_getvariables(D, vars, vel_names);
% t = clmcplot_getvariables(D, vars, time_name);
% 
% tan_slope = 100;
% 
% for i = 1:3
%     sg(:,i) = filtfilt(b, a, sg(:,i));
%     thd(:,i) = filtfilt(b, a, thd(:,i));
%     thd_fun{i} = [tanh(tan_slope * thd(:,i)) ones(length(thd(:,i)),1)];
% end
% 
% figure(2)
% for i = 1:3
%     subplot(3,1,i);
%     plot(t, sg(:,i), t, sg(:,i) - thd_fun{i} * param(:,i), ...
%         t, sg(:,i) - param(2,i), 'linewidth', 2);
%     line([t(1), t(end)], [0 0], 'linewidth', 2, 'color', 'k');
%     
%     [mean(sqrt( (sg(:,i)).^2 )) ...
%         mean(sqrt( (sg(:,i) - thd_fun{i} * param(:,i)) .^2 )) ...
%         mean(sqrt( (sg(:,i) - param(2,i)).^2 ))]
% end

function x = remove_position_offset(th, grid_offset, grid)

x = zeros(length(th), 1);

for i = 1:length(th)
    ind = find(grid > th(i), 1);
    if(~isempty(ind))
        x(i) = grid_offset(ind);
    else
        x(i) = grid_offset(end);
    end
end