clear all;

dir = 'savedData/iros_data/red_cup_friday';

file_ol = 'd00399';
file_fb = 'd00398';

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

[euler_ol(:,1) euler_ol(:,2) euler_ol(:,3)]  = quat2angle(q_ol,'ZXY');
[euler_fb(:,1) euler_fb(:,2) euler_fb(:,3)] = quat2angle(q_fb, 'ZXY');


%%planar positions and orientation
figure;
subplot(3,1,1);
plot(t_ol, x_ol(:,1), '--', t_fb, x_fb(:,1), 'linewidth', 4);
set(gca, 'fontsize', 20);
ylabel('X position [m]');
xlim([0 13]);
ylim([0.4 1.05]);

subplot(3,1,2);
plot(t_ol, x_ol(:,2), '--', t_fb, x_fb(:,2), 'linewidth', 4);
set(gca, 'fontsize', 20);
ylabel('Y position [m]');
xlim([0 13]);
ylim([0.25 0.8]);

subplot(3,1,3);
plot(t_ol, euler_ol(:,1)/pi*180, '--', t_fb, euler_fb(:,1)/pi*180, 'linewidth', 4);
set(gca, 'fontsize', 20);
ylabel('Z orientation [degrees]');
xlim([0 13]);
ylim([85 160]);

xlabel('Time [s]');


%%forces
figure;
subplot(3, 1, 1);
plot(t_fb, desF_fb(:,1), '--', t_fb, filter(b,a,F_fb(:,1)), t_ol, filter(b,a,F_ol(:,1)), 'linewidth', 4);
set(gca, 'fontsize', 20);
ylabel('X force [N]');
xlim([0 13]);
ylim([-2 0.5]);

subplot(3, 1, 2);
plot(t_fb, desF_fb(:,2), '--', t_fb, filter(b,a,F_fb(:,2)), t_ol, filter(b,a,F_ol(:,2)),'linewidth', 4);
set(gca, 'fontsize', 20);
ylabel('Y force [N]');
xlim([0 13]);
ylim([-2 0.5]);

subplot(3, 1, 3);
plot(t_fb, desF_fb(:,3), '--', t_fb, filter(b,a,F_fb(:,3)), t_ol, filter(b,a,F_ol(:,3)), 'linewidth', 4);
set(gca, 'fontsize', 20);
ylabel('Z torque [Nm]');
xlim([0 13]);
ylim([-0.1 0.4]);
xlabel(['Time [s]']);

figure;
fing_label = {'Right Finger', 'Middle Finger', 'Left Finger'};
for i = 1:3
    subplot(3,1,i);
    plot(t_ol, fingers_ol(:,i)*180/pi, '--', t_fb, fingers_fb(:,i)*180/pi, 'linewidth', 4);
    set(gca, 'fontsize', 20);
    ylabel(fing_label{i});
    xlim([0 13]);
    ylim([-0.01 130]);
end
xlabel('Time');

% 
% figure;
% for i = 1:3
%     subplot(3,1,i);
%     plot(t_ol, x_ol(:,i), t_fb, x_fb(:,i), 'linewidth', 4);
%     xlim([0 10]);
% end
% 
% 
% figure;
% for i = 1:3
%     subplot(3,1,i);
%     F_fb(:,i) = filter(b,a,F_fb(:,i));
%     plot(t_fb, desF_fb(:,i), t_fb, F_fb(:,i), 'linewidth', 4);
%     ylim([-2 0.5]);
%     xlim([0 10]);
% end
% 
% 
% 
% figure;
% [euler_ol(:,1) euler_ol(:,2) euler_ol(:,3)]  = quat2angle(q_ol,'ZXY');
% [euler_fb(:,1) euler_fb(:,2) euler_fb(:,3)] = quat2angle(q_fb, 'ZXY');
% 
% for i = 1:3
%     subplot(3,1,i);
%     plot(t_ol, euler_ol(:,i)/pi*180, t_fb, euler_fb(:,i)*180/pi, 'linewidth', 4);
% end
% 
% 
% figure;
% for i = 1:3
%     subplot(3,1,i);
%     plot(t_ol, fingers_ol(:,i)*180/pi, t_fb, fingers_fb(:,i)*180/pi);
% end
% 
% figure;
% for i = 1:3
%     subplot(3,1,i);
%     plot(t_ol, sg_ol(:,i), t_fb, sg_fb(:,i));
% end
