function [var_names var_des_names] = get_kinesthetic_teaching_var_names(filename)
% Get the variable names that should be recorded during kinesthetic teaching
%
% Input parameters
%   filename      : Save these names to this file (optional)
%
% Output parameters
%   var_names     : Names of the variables in a string array
%   var_des_names : Names of the desired variables in a string array


%-------------------------------------------------------------------------------
% 1) Generate desired variable names
var_des_names = {};

% First, include joint names
joint_names = get_joint_names;
for jj=1:length(joint_names)-4 % Exclude head
  var_des_names{end+1} = [ joint_names{jj} '_des_th' ];
  var_des_names{end+1} = [ joint_names{jj} '_des_thd' ];
  var_des_names{end+1} = [ joint_names{jj} '_des_thdd' ];
end

% Then, include end-effector names
end_eff_names = {
  'R_HAND_des_x','R_HAND_des_y','R_HAND_des_z',...
  'R_HAND_des_q0','R_HAND_des_q1','R_HAND_des_q2','R_HAND_des_q3',...
};

% Generate names for velocity and acceleration too
for ee=1:length(end_eff_names)
  var_des_names{end+1} = [ end_eff_names{ee}   '' ];
  var_des_names{end+1} = [ end_eff_names{ee}  'd' ];
  var_des_names{end+1} = [ end_eff_names{ee} 'dd' ];
end

%-------------------------------------------------------------------------------
% 2) Generate variable names (i.e. simply take out '_des' label)
var_names = var_des_names;
for vv=1:length(var_names)
  var_names{vv} = strrep(var_names{vv},'_des','');
end

%-------------------------------------------------------------------------------
% 3) Save to file if necessary
if (exist('filename','var'))
  fid = fopen(filename,'w');
  for vv=1:length(var_names)
    fprintf(fid,'%s\n',var_names{vv});
  end
  fclose(fid);
end