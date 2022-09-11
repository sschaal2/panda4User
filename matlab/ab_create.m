function A=ab_create(txt_file)
% takes as input a text file with dt, pos, vel, acc from 4 panda robots and
% converts this to clmcplot format

% load a template file to get variable names
load ab

% load data
A = load(txt_file);
[n,m] = size(A);

% the sampling time
dt = A(1,1);

freq = 1/dt;
A(:,1) = (dt:dt:dt*n)';

out_file = [txt_file '.traj'];

mrdplot_gen(A,vars,freq,out_file);