clear all;

file_ol = 'd00305';
file_fb = 'd00304';%'d00253';

[D_fb vars freq] = clmcplot_convert(file_fb);
[D_ol vars freq] = clmcplot_convert(file_ol);

names = {vars(:).name};

t_ol = D_ol(:,find(strcmp(names,'time')));
t_fb = D_fb(:,find(strcmp(names,'time')));

x_ol(:,1) = D_ol(:,find(strcmp(names, 'R_HAND_x')));
x_fb(:,1) = D_fb(:,find(strcmp(names, 'R_HAND_x')));

x_ol(:,2) = D_ol(:,find(strcmp(names, 'R_HAND_y')));
x_fb(:,2) = D_fb(:,find(strcmp(names, 'R_HAND_y')));

x_ol(:,3) = D_ol(:,find(strcmp(names, 'R_HAND_z')));
x_fb(:,3) = D_fb(:,find(strcmp(names, 'R_HAND_z')));

q_ol(:,1) = D_ol(:,find(strcmp(names, 'R_HAND_q0')));
q_ol(:,2) = D_ol(:,find(strcmp(names, 'R_HAND_q1')));
q_ol(:,3) = D_ol(:,find(strcmp(names, 'R_HAND_q2')));
q_ol(:,4) = D_ol(:,find(strcmp(names, 'R_HAND_q3')));

q_fb(:,1) = D_fb(:,find(strcmp(names, 'R_HAND_q0')));
q_fb(:,2) = D_fb(:,find(strcmp(names, 'R_HAND_q1')));
q_fb(:,3) = D_fb(:,find(strcmp(names, 'R_HAND_q2')));
q_fb(:,4) = D_fb(:,find(strcmp(names, 'R_HAND_q3')));

fingers_ol(:,1) = D_ol(:,find(strcmp(names, 'R_RF_th')));
fingers_ol(:,2) = D_ol(:,find(strcmp(names, 'R_MF_th')));
fingers_ol(:,3) = D_ol(:,find(strcmp(names, 'R_LF_th')));

fingers_fb(:,1) = D_fb(:,find(strcmp(names, 'R_RF_th')));
fingers_fb(:,2) = D_fb(:,find(strcmp(names, 'R_MF_th')));
fingers_fb(:,3) = D_fb(:,find(strcmp(names, 'R_LF_th')));

sg_ol(:,1) = D_ol(:,find(strcmp(names, 'R_RF_SG')));
sg_ol(:,2) = D_ol(:,find(strcmp(names, 'R_MF_SG')));
sg_ol(:,3) = D_ol(:,find(strcmp(names, 'R_LF_SG')));

sg_fb(:,1) = D_fb(:,find(strcmp(names, 'R_RF_SG')));
sg_fb(:,2) = D_fb(:,find(strcmp(names, 'R_MF_SG')));
sg_fb(:,3) = D_fb(:,find(strcmp(names, 'R_LF_SG')));


for i = 1:3
    subplot(3,1,i);
    plot(t_ol, x_ol(:,i), t_fb, x_fb(:,i));
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
