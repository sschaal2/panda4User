function [joint_mse, vel_mse, ufb_ratio] = compute_tracking_performances(dir, file)



joint_names = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7'};

[D vars freq] = clmcplot_convert(sprintf('%s/%s',dir,file));

names = {vars(:).name};

for i = 1:length(joint_names)
    
    pos = D(:,find(strcmp(names, sprintf('%s_th', joint_names{i}))));
    des_pos = D(:,find(strcmp(names, sprintf('%s_des_th', joint_names{i}))));
    
    joint_mse(i) = sqrt(mean( (pos-des_pos).^2));
    
    vel = D(:,find(strcmp(names, sprintf('%s_thd', joint_names{i}))));
    des_vel = D(:,find(strcmp(names, sprintf('%s_des_thd', joint_names{i}))));
    
    vel_mse(i) = sqrt(mean( (vel-des_vel).^2));

    torque = D(:,find(strcmp(names, sprintf('%s_u', joint_names{i}))));
    ufb = D(:,find(strcmp(names, sprintf('%s_ufb', joint_names{i}))));

    ufb_ratio(i) = mean(ufb.^2) / mean(torque.^2);
%    ufb_ratio(i) = mean(abs(ufb./torque))
    
end


figure(6)
bar([joint_mse',vel_mse',ufb_ratio'])
legend('q','qd','u_ratio')
