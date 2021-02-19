clear all;

dir = 'savedData/iros_data/maglite';
%dir = 'savedData/iros_data/red_cup_friday';

%file_ol = 'd00257';
%file_fb = 'd00261';%'d00253';
%file_fb2 = 'd00271';

file_ol = 'd00289';
file_fb = 'd00285';%'d00253';
file_fb2 = 'd00300';

[D_fb vars freq] = clmcplot_convert(sprintf('%s/%s',dir,file_fb));
[D_fb2 vars freq] = clmcplot_convert(sprintf('%s/%s',dir,file_fb2));
[D_ol vars freq] = clmcplot_convert(sprintf('%s/%s',dir,file_ol));

varnames = {'time', 'R_HAND_x', 'R_HAND_y', 'R_HAND_z'};
[t_ol, x_ol(:,1), x_ol(:,2), x_ol(:,3)] = clmcplot_getvariables(D_ol, vars, varnames);
[t_fb, x_fb(:,1), x_fb(:,2), x_fb(:,3)] = clmcplot_getvariables(D_fb, vars, varnames);
[t_fb2, x_fb2(:,1), x_fb2(:,2), x_fb2(:,3)] = clmcplot_getvariables(D_fb2, vars, varnames);


plot(x_ol(:,1), x_ol(:,2), x_fb(:,1), x_fb(:,2), x_fb2(:,1), x_fb2(:,2),'linewidth', 4);
xlim([0.5 1.05]);
ylim([0.3 0.85]);