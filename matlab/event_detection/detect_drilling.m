filename = 'd00039';
varnames = {'R_HAND_force_X', 'R_HAND_force_Y', 'R_HAND_force_Z', 'R_HAND_torque_X', 'R_HAND_torque_Y', 'R_HAND_torque_Z'};

[D vars freq] = clmcplot_convert(filename);
force = clmcplot_getvariables(D, vars, varnames);
t = clmcplot_getvariables(D, vars, {'time'});

for i = 1:6
    force(:,i) = force(:,i) - mean(force(:,i));
end


force1 = force(1:4800,:);
force2 = force(6000:10000,:);



color = ['b', 'r', 'g'];

figure;
for i = 1:3
    myFFTplot(freq, force1(:,i), color(i));
    hold on;
end
ylim([0 0.01]);

figure;
for i = 1:3
    myFFTplot(freq, force2(:,i), color(i));
    hold on;
end
ylim([0 0.01]);

figure;
tot1 = sqrt(force1(:,1).^2 + force1(:,2).^2 + force1(:,3).^2);
tot2 = sqrt(force2(:,1).^2 + force2(:,2).^2 + force2(:,3).^2);

tot1 = tot1 - mean(tot1);
tot2 = tot2 - mean(tot2);


subplot(2, 1, 1);
myFFTplot(freq, tot1);
ylim([0 0.01]);

subplot(2,1,2);
myFFTplot(freq, tot2);
ylim([0 0.01]);
