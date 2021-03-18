function sys_test_maglite

firsttime = 0;

dir = 'savedData/iros_data/sys_test_maglite';


%%all the files from the systematic test
files = {'d00308', 'd00309', 'd00310', 'd00311', 'd00312', 'd00315', 'd00316',...
    'd00317', 'd00318','d00319', 'd00320', 'd00321', 'd00322', 'd00323', 'd00324',...
    'd00326', 'd00327', 'd00328', 'd00333', 'd00334', 'd00335', 'd00337',...
    'd00338', 'd00343', 'd00348', 'd00349', 'd00339', 'd00411', 'd00412'};

%%the corresponding grid they try to reach
grid = [4 0; 3 0; 2 0; 1 0; -1 0;...
    -2 -1; -1 -1; 0 -1; 1 -1; 2 -1; 3 -1; 4 -1;...
    4 -2; 3 -2; 2 -2; 1 -2; 0 -2; -1 -2;...
    0 -3; 1 -3; 2 -3; 3 -3; 4 -3;...
    2 -4;...
    0 1; 1 1;...
    5 -3;
    -1 1; 2 1] * 0.04;

%%the grid for the open loop case (where it succeeded
grid_ol = [-1 0; 0 0; 1 0; 2 0; 0 1; 1 1; 2 1] * 0.04;

file_nofb = 'd00356';

if(firsttime)
    gather_data(files, dir,file_nofb);
end
load('temp');

%%create a mesh grid
[total_grid_x total_grid_y total_grid_z] = create_meshgrid(grid, grid_ol);


%%find the alignement between the base coordinates and the test square
[rot_angle bias(1) bias(2)] = find_alignement(x,y,grid);
rot_angle = -0.9;
rot_matrix = [cos(rot_angle) -sin(rot_angle); sin(rot_angle) cos(rot_angle)];


%%rotate the grids
grid = grid;
grid_ol = grid_ol;

%%plots
figure;
for i = 1:length(files)
    x{i} = x{i} - bias(1);
    y{i} = y{i} - bias(2);
    plot(x{i}, y{i});
    %plot(x{i}(end), y{i}(end), 'bx', 'linewidth',1);
    hold on;
end

%plot(grid(:,1), grid(:,2), 'gx', 'linewidth', 4);
plot(0,0,  'kx', 'linewidth', 4);

xlim([-0.45 0.45]);
ylim([-0.75 0.15]);

plot(x0, y0, 'r', 'linewidth', 2);

set(gca, 'fontsize', 20);
xlabel('x coordinate [m]');
ylabel('y coordinate [m]');

set(gca, 'DataAspectRatio', [1 1 1]);
set(gca, 'YAxisLocation', 'right');

%%%%
%figure;
ax2 = axes('position',[0.25 0.25 0.35 0.35]);
temp = [reshape(total_grid_x,[],1) reshape(total_grid_y,[],1)];

total_grid_x = reshape(temp(:,1),size(total_grid_z));
total_grid_y = reshape(temp(:,2),size(total_grid_z));

%contourf(total_grid_x, total_grid_y, total_grid_z, 2);
%hold on;
plot(grid_ol(:,1), grid_ol(:,2), 'rx', 'linewidth', 4);
hold on;
plot(grid(:,1), grid(:,2), 'gx', 'linewidth', 4);
plot(0, 0, 'kx', 'linewidth', 4);
xlim([-0.1 0.22]);
ylim([-0.22 0.1]);

set(gca, 'fontsize', 20);
set(gca, 'DataAspectRatio', [1 1 1]);
%xlabel('x coordinate [m]');
%ylabel('y coordinate [m]');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [gridx, gridy, gridz] = create_meshgrid(grid, grid_ol)

%%make a mesh grid
[gridx, gridy] = meshgrid( (-2:1:5)*0.04, (-4:1:1)*0.04);
gridz = zeros(size(gridx));

%%fill the z dimension with 1s for the successful fb grasp
for i = 1:length(grid)
    idx = find(gridx == grid(i,1));
    idy = find(gridy == grid(i,2));
    id = intersect(idx, idy);
    if(~isempty(id))
        gridz(id) = 1;
    end
end

%%fill z with 2 for the ol grasp
for i = 1:length(grid_ol)
    idx = find(gridx == grid_ol(i,1));
    idy = find(gridy == grid_ol(i,2));
    id = intersect(idx, idy);
    if(~isempty(id))
        gridz(id) = 2;
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function gather_data(files, dir,file_nofb)

%%create the open loop trajectory
[D vars freq] = clmcplot_convert(sprintf('%s/%s', dir, file_nofb));
names = {vars(:).name};

x0 = D(:,find(strcmp(names,'R_HAND_des_x')));
y0 = D(:,find(strcmp(names,'R_HAND_des_y')));

for i = 1:length(files)
    
    [D vars freq] = clmcplot_convert(sprintf('%s/%s', dir, files{i}));
    
    x{i} = D(:,find(strcmp(names,'R_HAND_x'))) - x0(end);
    y{i} = D(:,find(strcmp(names,'R_HAND_y'))) - y0(end);
end

x0 = x0 - x0(end);
y0 = y0 - y0(end);

save('temp', 'x', 'y', 'x0', 'y0');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle, offset_x, offset_y] = find_alignement2(x,y,grid,x0,y0)

%%create regression matrix
reg = zeros(2*length(x)+2,4);
b = zeros(2*length(x)+2,1);

for i = 1:length(x)
    reg(2*(i-1)+1, 1) = x{i}(end);
    reg(2*(i-1)+1, 2) = y{i}(end);
    reg(2*(i-1)+1, 3) = 1;
    
    reg(2*(i-1)+2, 1) = y{i}(end);
    reg(2*(i-1)+2, 2) = -x{i}(end);
    reg(2*(i-1)+2, 4) = 1;
    
    b(2*(i-1)+1) = grid(i,1);
    b(2*(i-1)+2) = grid(i,2);
end
reg(2*length(x)+1, 1) = x0(end);
reg(2*length(x)+1, 2) = y0(end);
reg(2*length(x)+1, 3) = 1;

reg(2*length(x)+2, 1) = y0(end);
reg(2*length(x)+2, 2) = -x0(end);
reg(2*length(x)+2, 4) = 1;

b(2*length(x)+1) = 0;
b(2*length(x)+2) = 0;

size(reg)
size(grid)

parms = pinv(reg) * b

offset_x = parms(3);
offset_y = parms(4);
angle = (acos(parms(1)) + asin(parms(2)))/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [angle, offset_x, offset_y] = find_alignement(x,y,grid)

angle = 0.8;

%x0 = [angle offset_x offset_y];
x0 = angle;

%%create the end point matrix
for i = 1:length(x)
    A(i,1) = x{i}(end);
    A(i,2) = y{i}(end);
end

options = optimset('TolFun',1e-15, 'TolX', 1e-15);
f = @(param)( error_estimate(A, grid, angle));

[xend feval] = fminunc(f, x0, options);
xend
feval
angle = xend(1);
offset_x = 0.01;%xend(2);
offset_y = 0.01;%xend(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function er = error_estimate(A, grid, param)

rot = [cos(param(1)) -sin(param(1)); sin(param(1)) cos(param(1))];

%er = [A(:,1) - param(2) A(:,2) - param(3)] * rot - grid;
er = [A(:,1) A(:,2)] * rot - grid;

er = sqrt(sum(sum(er.^2)));