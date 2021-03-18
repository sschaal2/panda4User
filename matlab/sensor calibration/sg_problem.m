clear all;

sg_names = {'R_RF_SG', 'R_MF_SG', 'R_LF_SG'};
joint_names = {'R_RF_th', 'R_MF_th', 'R_LF_th'};
torque_names = {'R_RF_u', 'R_MF_u', 'R_LF_u'};

[D_new vars freq] = clmcplot_convert('d00631');
[sg_new(:,1) sg_new(:,2) sg_new(:,3)] = clmcplot_getvariables(D_new, vars, sg_names);
[th_new(:,1), th_new(:,2), th_new(:,3)] = clmcplot_getvariables(D_new, vars, joint_names);
[torque_new(:,1), torque_new(:,2), torque_new(:,3)] = clmcplot_getvariables(D_new, vars, torque_names);
time_new = clmcplot_getvariables(D_new, vars, {'time'});

[D_old vars freq] = clmcplot_convert('d00640');
[sg_old(:,1) sg_old(:,2) sg_old(:,3)] = clmcplot_getvariables(D_old, vars, sg_names);
[th_old(:,1) th_old(:,2) th_old(:,3)] = clmcplot_getvariables(D_old, vars, joint_names);
[torque_old(:,1) torque_old(:,2) torque_old(:,3)] = clmcplot_getvariables(D_old, vars, torque_names);
time_old = clmcplot_getvariables(D_old, vars, {'time'});


figure(1);

subplot(3,1,1);
plot(time_old, sg_old, 'linewidth', 4);
set(gca, 'fontsize', 30);
ylabel('SG value [Nm]');
ylim([-0.16 0.1]);

subplot(3,1,2);
plot(time_old, th_old, 'linewidth', 4);
set(gca, 'fontsize', 30);
ylabel('joint pos [rad]');
ylim([0 2]);

subplot(3,1,3);
plot(time_old, torque_old, 'linewidth', 4);
set(gca, 'fontsize', 30);
ylabel('command [Nm]');
xlabel('Time [s]');
ylim([-0.02 0.02]);

figure(2);

subplot(3,1,1);
plot(time_new, sg_new, 'linewidth', 4);
set(gca, 'fontsize', 30);
ylabel('SG value [Nm]');
ylim([-0.16 0.1]);

subplot(3,1,2);
plot(time_new, th_new, 'linewidth', 4);
set(gca, 'fontsize', 30);
ylabel('joint pos [rad]');
ylim([0 2]);

subplot(3,1,3);
plot(time_new, torque_new, 'linewidth', 4);
set(gca, 'fontsize', 30);
ylabel('command [Nm]');
xlabel('Time [s]');
ylim([-0.02 0.02]);

