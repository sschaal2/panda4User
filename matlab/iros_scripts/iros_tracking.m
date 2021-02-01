clear all;

dir = 'savedData/iros_data/red_cup_friday';

file_ol = 'd00399';
file_fb = 'd00398';%'d00253';

[D_fb vars freq] = clmcplot_convert(sprintf('%s/%s',dir,file_fb));
[D_ol vars freq] = clmcplot_convert(sprintf('%s/%s',dir,file_ol));

[b a] = butter(2, 0.05);

varnames_x = {'R_HAND_x', 'R_HAND_y', 'R_HAND_z'};
varnames_q = {'R_HAND_q0', 'R_HAND_q1', 'R_HAND_q2', 'R_HAND_q3'};
varnames_fing = {'R_RF_th', 'R_MF_th', 'R_LF_th'};
varnames_sg = {'R_RF_SG', 'R_MF_SG', 'R_LF_SG'};
varnames_F = {'R_HAND_force_X', 'R_HAND_force_Y', 'R_HAND_torque_Z'};
varnames_des_F = {'R_HAND_des_force_X', 'R_HAND_des_force_Y', 'R_HAND_des_torque_Z'};

t_ol = clmcplot_getvariables(D_ol, vars, {'time'});
t_fb = clmcplot_getvariables(D_fb, vars, {'time'});

x_ol = clmcplot_getvariables(D_ol, vars, varnames_x);
x_fb = clmcplot_getvariables(D_fb, vars, varnames_x);

q_ol = clmcplot_getvariables(D_ol, vars, varnames_q);
q_fb = clmcplot_getvariables(D_fb, vars, varnames_q);

fingers_ol = clmcplot_getvariables(D_ol, vars, varnames_fing);
fingers_fb = clmcplot_getvariables(D_fb, vars, varnames_fing);

sg_ol = clmcplot_getvariables(D_ol, vars, varnames_sg);
sg_fb = clmcplot_getvariables(D_fb, vars, varnames_sg);

F_fb = clmcplot_getvariables(D_fb, vars, varnames_F);
F_ol = clmcplot_getvariables(D_ol, vars, varnames_F);

desF_fb = clmcplot_getvariables(D_fb, vars, varnames_des_F);
desF_ol = clmcplot_getvariables(D_ol, vars, varnames_des_F);

for i = 1:3
    subplot(3,1,i);
    plot(t_ol, x_ol(:,i), t_fb, x_fb(:,i), 'linewidth', 4);
    xlim([0 10]);
end


figure;
for i = 1:3
    subplot(3,1,i);
    F_fb(:,i) = filter(b,a,F_fb(:,i));
    plot(t_fb, desF_fb(:,i), t_fb, F_fb(:,i), 'linewidth', 2);
    ylim([-2 0.5]);
    xlim([0 10]);
end


[x_ol y_ol z_ol]  = quat2angle(q_ol,'XYZ');
[x_fb y_fb z_fb] = quat2angle(q_fb, 'XYZ');

figure;
subplot(3,1,1);
plot(t_ol, x_ol*180/pi, t_fb, x_fb*180/pi);
ylim([-100 0]);
subplot(3,1,2);
plot(t_ol, y_ol*180/pi, t_fb, y_fb*180/pi);
ylim([-50 50]);
subplot(3,1,3);
plot(t_ol, z_ol*180/pi, t_fb, z_fb*180/pi);
ylim([0 100]);


figure;
for i = 1:3
    subplot(3,1,i);
    plot(t_ol, fingers_ol(:,i)*180/pi, t_fb, fingers_fb(:,i)*180/pi);
end
figure;
for i = 1:3
    subplot(3,1,i);
    plot(t_ol, sg_ol(:,i), t_fb, sg_fb(:,i));
    line([0 t_ol(end)], [-0.07 -0.07]);
end
