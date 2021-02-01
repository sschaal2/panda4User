[D vars freq] = clmcplot_convert('d00674');
load_cell_names = {'R_HAND_force_Z'};

f = clmcplot_getvariables(D, vars, load_cell_names);
t = clmcplot_getvariables(D, vars, {'time'});

window = 30;
threshold = 20;

len = length(f);

events = [];

for i = 1:len-window
    
    temp = f(i:i+window);
    f_max = max(temp);
    f_min = min(temp);
    
    if(f_max - f_min > threshold)
        events = [events; t(i)];
    end
end
events

if(~isempty(events))
    plot(t, f);
    hold on;
    for i = 1:length(events)
        line([events(i) events(i)], [0 60], 'linestyle',':', 'color', 'k');
    end
end