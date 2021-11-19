dir = '.';
fileoffset = 'd00106';

file = 'd00106';

offset = computeFTOffset(fileoffset, dir)

mass = (1.18+0.136/2);

[F Fworld] = computeFT_force(file, dir, offset);
figure;
for i = 1:3
    subplot(3,1,i);
    plot(Fworld(:,i));
    %hold on;
    %plot(F(:,i), 'r');
end
line([0 length(Fworld(:,i))], -mass *9.81 * [1 1] , 'linewidth', 2);


fx = median(Fworld(:,1));
fy = median(Fworld(:,2));
fz = median(Fworld(:,3));
ftot = sqrt(fx^2+fy^2+fz^2);
line([0 length(Fworld(:,i))], -ftot * [1 1] , 'linewidth', 2, 'color', 'r');


% 
% 
% [F1 Fworld1] = computeFT_force('d00094', dir);
% [F2 Fworld2] = computeFT_force('d00095', dir);
% 
% norm(mean(F1-F2))
% norm(mean(Fworld1- Fworld2))
