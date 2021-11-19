function F_offset = computeFTOffset(file, dir, Freal_w)

if(nargin<3)
    Freal_w = [0; 0; -(1.18+0.136/2)*9.81];
end

[D vars freq] = clmcplot_convert(sprintf('%s/%s', dir, file));

names = {vars(:).name};

Fx = D(:,find(strcmp(names, 'R_LC_FX')));
Fy = D(:,find(strcmp(names, 'R_LC_FY')));
Fz = D(:,find(strcmp(names, 'R_LC_FZ')));

q0 = D(:,find(strcmp(names, 'R_HAND_q0')));
q1 = D(:,find(strcmp(names, 'R_HAND_q1')));
q2 = D(:,find(strcmp(names, 'R_HAND_q2')));
q3 = D(:,find(strcmp(names, 'R_HAND_q3')));

F = [Fx Fy Fz];

for i = 1:length(Fx)
    alpha = -pi/2;
    R2 = [cos(alpha) sin(alpha) 0;
        -sin(alpha) cos(alpha) 0;
        0 0 1];
    R = get_rotation_matrix(q0(i), q1(i), q2(i), q3(i));
    Freal_local(i,:) = R2' * R' * Freal_w;
end


a = F - Freal_local;

F_offset = median(a);
