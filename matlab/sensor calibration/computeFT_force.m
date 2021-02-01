function [F Fworld] = computeFT_force(file, dir, offset)

if(nargin<3)
    offset = [0 0 0]
end


[D vars freq] = clmcplot_convert(sprintf('%s/%s', dir, file));

names = {vars(:).name};

Fx = D(:,find(strcmp(names, 'R_LC_FX'))) - offset(1);
Fy = D(:,find(strcmp(names, 'R_LC_FY'))) - offset(2);
Fz = D(:,find(strcmp(names, 'R_LC_FZ'))) - offset(3);

F = [Fx Fy Fz];


q0 = D(:,find(strcmp(names, 'R_HAND_q0')));
q1 = D(:,find(strcmp(names, 'R_HAND_q1')));
q2 = D(:,find(strcmp(names, 'R_HAND_q2')));
q3 = D(:,find(strcmp(names, 'R_HAND_q3')));

for i = 1:length(Fx)
    R = get_rotation_matrix(q0(i), q1(i), q2(i), q3(i));
    alpha = pi/2;
    R2 = [cos(alpha) sin(alpha) 0;
        -sin(alpha) cos(alpha) 0;
        0 0 1];
    Fworld(i,:) =  R * R2 * [Fx(i); Fy(i); Fz(i)];
end
