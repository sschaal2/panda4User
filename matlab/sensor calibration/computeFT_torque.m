
[D vars freq] = clmcplot_convert('d00500');

names = {vars(:).name};

%Tx = D(:,find(strcmp(names, 'R_HAND_torque_X')));
%Ty = D(:,find(strcmp(names, 'R_HAND_torque_Y')));
%Tz = D(:,find(strcmp(names, 'R_HAND_torque_Z')));

Tx = D(:,find(strcmp(names, 'R_LC_MX')));
Ty = D(:,find(strcmp(names, 'R_LC_MY')));
Tz = D(:,find(strcmp(names, 'R_LC_MZ')));

mean(Tz.^2)
T = sqrt(Tx.^2 + Ty.^2 + Tz.^2);
mean(T)