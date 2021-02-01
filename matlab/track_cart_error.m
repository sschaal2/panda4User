[D vars freq] = clmcplot_convert('d00571');
hand_pos = clmcplot_getvariables(D, vars, {'R_HAND_x','R_HAND_y','R_HAND_z'});
des_hand_pos = clmcplot_getvariables(D, vars, {'d_R_HAND_des_x','d_R_HAND_des_y','d_R_HAND_des_z'});


mid = floor(length(hand_pos(:,1))/2);

error1 = sqrt(sum( (hand_pos-des_hand_pos).^2, 2));

figure;
plot(error1(1:mid), 'b');
hold on;
plot(error1(mid:end), 'r');
