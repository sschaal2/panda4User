[D vars freq] = clmcplot_convert('d00657');

load_cell_names = {'R_LC_FX','R_LC_FY','R_LC_FZ','R_LC_MX','R_LC_MY','R_LC_MZ'};

f = clmcplot_getvariables(D, vars, load_cell_names);
t = clmcplot_getvariables(D, vars, {'time'});

time_win = 10; %seconds

med1 = median(f(1:300*time_win,:));
med2 = median(f(end-300*time_win:end,:)) - med1;

ylabels = {'FX [N]', 'FY [N]', 'FZ [N]', 'MX [Nm]', 'MY [Nm]', 'MZ [Nm]'};

for i = 1:6
    figure(i);
    hold on;
    plot(t,f(:,i) - med1(i), 'linewidth', 2,'color','b');
    line([0 120], [0 0], 'linestyle', '--', 'linewidth', 2, 'color', 'k');
    ylabel(ylabels{i});
    xlabel('Time [s]');
    saveas(gcf,sprintf('%s_hyst_test2', load_cell_names{i}), 'png');
end
