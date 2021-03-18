function [filename] = append_close_gripper(filename,duration_closing,open_angles,close_angles,finger_joint_names);
% Append a closing movement of the gripper to an existing mrd file.
%
% During the movement of the end-effector, the gripper is opened.
% The closing of the gripper is a minimum jerk movement in joint space.
%
% Input parameters
%   filename           : filename of the input mrd file (default: .last_data)
%   duration_closing   : duration in which the gripper closes
%   open_angles        : initial angles of the open gripper
%   close_angles       : final angles of the closed gripper
%   finger_joint_names : names of the fingers (default: Barret hand)
%
% Output parameters
%   filename         : the filename of the mrd file to which the new data was saved

if (~exist('filename','var') || isempty(filename))
  if exist('.last_data','file')
    filename = sprintf('d%05d',load('.last_data')-1);
  else
    fprintf('ERROR: You must provide a filename.\n');
    return;
  end
end

if ~exist('duration','var'),     duration_closing = 3.0;                 end
if ~exist('close_angles','var'), close_angles     = [0.101 2.0 2.0 2.0]; end
if ~exist('open_angles','var'),  open_angles      = [0.100 0.6 0.6 0.6]; end

% Default: use WAM joints
if ~exist('finger_joint_names','var')
  finger_joint_names = {'R_FR_des_th', 'R_RF_des_th', 'R_MF_des_th', 'R_LF_des_th'};
end

% Read mrd file, determine some parameters, and collect variable names
fprintf('Reading from file "%s"\n',filename);
addpath ~clmc/matlab/utilities/
[D, vars, frequency] = clmcplot_convert(filename);
traj_length_input = size(D,1);
duration_input    = traj_length_input/frequency;

var_names = cell(1,length(vars));
for vv=1:length(vars), var_names{vv} = vars(vv).name; end

% Compute new duration in seconds and ticks
traj_length_closing = duration_closing*frequency;
traj_length_output  = traj_length_input+traj_length_closing;
duration_output     = duration_input+duration_closing;

% Copy the final entries, i.e. the arm rests while the gripper is closing
last_values = D(end,:);
D = [D; repmat(last_values,traj_length_closing,1)];


units = {'rad','rad/s','rad/s^2'};

for ff=1:length(finger_joint_names)
  
  % Generate a minjerk trajectory for this finger
  finger_traj = open_angles(ff)*ones(traj_length_output,1);  
  finger_traj(end-traj_length_closing+1:end) = min_jerk([open_angles(ff) close_angles(ff)]',traj_length_closing,[],[],[]);

  append_dd = '';
  for differentiate=0:2 % add rad, rad/s and rad/s^2
    finger_name = [ finger_joint_names{ff} append_dd];
    index = find(strcmp(finger_name ,var_names));
    
    if (isempty(index))
      % Append data
      %fprintf('  Appending data for "%s".\n',finger_name)
      D = [D finger_traj];
      vars(end+1).name = finger_name;
      vars(end+1).unit = units{differentiate+1};
      
    else
      % Replace data
      %fprintf('  Replacing data for "%s".\n',finger_name)
      D(:,index) = finger_traj;
      
    end
    
    % Differentiate
    finger_traj = diffnc(finger_traj,1/frequency);
    append_dd = [ append_dd 'd'];
    
  end
end

filename = [ strrep(filename,'.mrd','') '_closegripper.mrd' ];
fprintf('Saving to file "%s"\n',filename);
clmcplot_gen(D,vars,frequency,filename)

% Finally issue warning for some DMP implementations
same_angle_indices = find(close_angles==open_angles);
if (~isempty(same_angle_indices))
  fprintf('WARNING: Some intial angles are the same as the final angles. This might cause problems when learning a DMP.\n');
end

%-------------------------------------------------------------------------------
% Plot results

% Update var_names array (names might have been appended)
var_names = cell(1,length(vars));
for vv=1:length(vars), var_names{vv} = vars(vv).name; end


% Time for x-axis
time = linspace(0,duration_output,traj_length_output);

colors = [1 0.6 0.6; 0.6 1 0.6; 0.6 0.6 1; 0.6 1 1; 1 0.6 1; 1 1 0.6];
colors = [colors; 0.6*colors; 0.3*colors];
n_colors = length(colors);

figure(5);
legend_labels = {{},{},{}};
for vv=1:length(var_names)
  if (regexp(var_names{vv},'dd$'))
    sp = 3;
  elseif (regexp(var_names{vv},'d$'))
    sp = 2;
  else
    sp = 1;
  end
  
  subplot(3,1,sp)
  legend_labels{sp}{end+1} = strrep(var_names{vv},'_','-');
  
  linewidth=1;
  for jj=1:length(finger_joint_names)
    if (findstr(finger_joint_names{jj},var_names{vv}))
      linewidth=3; % Plot fingers thicker
    end
  end
    
  plot(time,D(:,vv),'-k','LineWidth',linewidth,'Color',1.0*colors(mod(vv,n_colors)+1,:));
  hold on
  
end

for sp=1:3
  subplot(3,1,sp)
  plot([duration_input duration_input],ylim,'-k')
  hold off
  legend(legend_labels{sp})
end

end