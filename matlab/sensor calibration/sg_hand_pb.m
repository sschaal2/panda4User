clear all;
dir = '';

sg_names = {'R_RF_SG', 'R_MF_SG', 'R_LF_SG'};

[D_new vars freq] = clmcplot_convert(sprintf('%s/%s',dir,'d00640'));
[sg_new(:,1) sg_new(:,2) sg_new(:,3)] = clmcplot_getvariables(D_new, vars, sg_names);
time_new = clmcplot_getvariables(D_new, vars, {'time'}) - 500/300;

[D_old vars freq] = clmcplot_convert(sprintf('%s/%s',dir,'d00632'));
[sg_old(:,1) sg_old(:,2) sg_old(:,3)] = clmcplot_getvariables(D_old, vars, sg_names);
time_old = clmcplot_getvariables(D_old, vars, {'time'});


figure(1);
for i = 2:2
    subplot(3,1,i);
    hold on;
    plot(time_old, sg_old(:,i) - mean(sg_old(:,i)), 'g', 'linewidth', 1);
    iqr(sg_old(:,i))
    %xlim([0 2500]);
    ylim([-0.02 0.02]);
    set(gca, 'fontsize', 20);
    ylabel('Strain gages [Nm]');
    xlabel('Time [s]');
end



figure(1);
for i = 2:2
    subplot(3,1,i);
    plot(time_new, sg_new(:,i) - mean(sg_new(:,i)), 'b', 'linewidth', 1);
    iqr(sg_new(500:end, i))
    xlim([0 7]);
end



[D_old vars freq] = clmcplot_convert(sprintf('%s/%s',dir,'d00400'));
clear sg_old;
[sg_old(:,1) sg_old(:,2) sg_old(:,3)] = clmcplot_getvariables(D_old, vars, sg_names);
mean(sg_old)


[D_old vars freq] = clmcplot_convert(sprintf('%s/%s',dir,'d00407'));
clear sg_old;
[sg_old(:,1) sg_old(:,2) sg_old(:,3)] = clmcplot_getvariables(D_old, vars, sg_names);
mean(sg_old)

[D_old vars freq] = clmcplot_convert(sprintf('%s/%s',dir,'d00413'));
clear sg_old;
[sg_old(:,1) sg_old(:,2) sg_old(:,3)] = clmcplot_getvariables(D_old, vars, sg_names);
mean(sg_old)
