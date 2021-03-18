%files1 = {'d00034', 'd00037', 'd00040', 'd00044', 'd00053', 'd00057', 'd00061'};
%files2 = {'d00035', 'd00038', 'd00041', 'd00045', 'd00054', 'd00058', 'd00062'};
%files3 = {'d00036', 'd00039', 'd00042', 'd00046', 'd00055', 'd00060', 'd00063'};

files1 = {'d00165'};
files2 = {'d00261'};
files3 = {'d00267'};

dir = '.';


for i = 1:length(files1)

    [joint_mse1(i,:) vel_mse1(i,:) ufb_ratio1(i,:)] = compute_tracking_performances(dir, files1{i})
    
end

for i = 1:length(files2)

    [joint_mse2(i,:) vel_mse2(i,:) ufb_ratio2(i,:)] = compute_tracking_performances(dir, files2{i})
    
end

for i = 1:length(files3)

    [joint_mse3(i,:) vel_mse3(i,:) ufb_ratio3(i,:)] = compute_tracking_performances(dir, files3{i})
    
end

for i = 1:7
    figure(1);
    subplot(2,4,i);
    plot(joint_mse1(:,i), 'bx', 'linewidth',10);
    hold on;
    plot(joint_mse2(:,i), 'rx', 'linewidth',10);
    plot(joint_mse3(:,i), 'gx', 'linewidth',10);
    ylim([0 0.04]);
    
    figure(2);
    subplot(2,4,i);
    plot(vel_mse1(:,i), 'bx', 'linewidth',10);
    hold on;
    plot(vel_mse2(:,i), 'rx', 'linewidth',10);
    plot(vel_mse3(:,i), 'gx', 'linewidth',10);
    ylim([0 0.18]);

    
    figure(3);
    subplot(2,4,i);
    plot(ufb_ratio1(:,i), 'bx', 'linewidth',10);
    hold on;
    plot(ufb_ratio2(:,i), 'rx', 'linewidth',10);
    plot(ufb_ratio3(:,i), 'gx', 'linewidth',10);
    ylim([0 1.15]);
end