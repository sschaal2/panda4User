function [filename] = crop_and_filter_mrd(t_range,filename)
% Read an mrd file, crop it to a specified length, and filter it with a
% Butterworth filter
%
%   t_range          - time range to include, rest will be cropped (in seconds)
%   filename         - filename (default: read from .lastdata file)
% 
%  example: crop_and_filter_mrd([0 1.5],'file.mrd')

if (~exist('filename','var') || isempty(filename))
  if exist('.last_data','file')
    filename = sprintf('d%05d',load('.last_data')-1);
  else
    fprintf('ERROR: You must provide a filename.\n');
    return;
  end
end


%------------------------------------------------------------------
% Load data from mrd file
addpath ~clmc/matlab/utilities/
fprintf('Reading from file "%s"\n',filename);
[D_in, vars_in, frequency] = clmcplot_convert(filename);
traj_length_in = size(D_in,1);                % Number of samples
duration_in    = traj_length_in/frequency; % Duration in s
time_in        = linspace(0,duration_in,traj_length_in)'; % Time axis

%------------------------------------------------------------------
% Crop the data
t_0 = 0;
t_end = duration_in;
if (exist('t_range','var') && ~isempty(t_range))
  t_0 = t_range(1);
  t_end = t_range(2); 
end

D_crop          = D_in((t_0*frequency)+1:t_end*frequency,:);
traj_length_out = size(D_crop,1);           % Number of samples
time_out        = linspace(t_0,t_end,traj_length_out)'; % Time axis
duration_out    = traj_length_out/frequency; % Duration in s


%------------------------------------------------------------------
% Flip quaternion sign if necessary
quat_indices = [];
for qq=0:3
  for vv=1:length(vars_in)
    if (regexp(vars_in(vv).name,sprintf('q%d$',qq)))
      quat_indices(end+1) = vv;
    end
  end
end

if (length(quat_indices)==4) % All four quaternion components must be available
  D_crop(:,quat_indices) = fixquaternionsign(D_crop(:,quat_indices));
end


%------------------------------------------------------------------
% Filter the data

% Get the indices of the position, velocity and acceleration data.
POS=1; VEL=2; ACC=3;
indices = {[],[],[]};
for vv=1:length(vars_in)
  if (regexp(vars_in(vv).name,'dd$'))
    indices{ACC}(end+1) = vv;
  elseif (regexp(vars_in(vv).name,'d$'))
    indices{VEL}(end+1) = vv;
  else
    indices{POS}(end+1) = vv;
  end
end

fc_factor = 10; % Filter coefficient
% Filter only needs position data, but returns position, velocity and
% acceleration as follows: pos vel acc pos vel acc ...
D_filt = scale_filt([time_out, D_crop(:,indices{POS})],traj_length_out,fc_factor);

% Make new vars array with names and units
for ii=1:length(indices{POS})
  pos_index = indices{POS}(ii);  % Index in original vars_in array 
  new_pos_index = 3*(ii-1);      % Index in new vars_filt array 

  % Names
  vars_filt(new_pos_index+1).name = [ vars_in(pos_index).name '' ];
  vars_filt(new_pos_index+2).name = [ vars_in(pos_index).name 'd' ];
  vars_filt(new_pos_index+3).name = [ vars_in(pos_index).name 'dd' ];
  
  % Units
  if (strcmp(vars_in(pos_index).unit,'-'))
    % Unit free remains unit free
    vars_filt(new_pos_index+1).unit = '-';
    vars_filt(new_pos_index+2).unit = '-';
    vars_filt(new_pos_index+3).unit = '-';
  else
    vars_filt(new_pos_index+1).unit = [ vars_in(pos_index).unit '' ];
    vars_filt(new_pos_index+2).unit = [ vars_in(pos_index).unit '/s' ];
    vars_filt(new_pos_index+3).unit = [ vars_in(pos_index).unit '/s^2' ];
  end
  
end

%------------------------------------------------------------------
% Save to new mrd file
filename = strrep(filename,'.mrd','_filtered_cropped.mrd');
%filename = 'demonstrated.mrd';
fprintf('Saving to file "%s"\n',filename);
clmcplot_gen(D_filt,vars_filt,frequency,filename)

%------------------------------------------------------------------
% Plot the original and filtered data
colors = [1 0.6 0.6; 0.6 1 0.6; 0.6 0.6 1; 0.6 1 1; 1 0.6 1; 1 1 0.6];
colors = [colors; 0.6*colors; 0.3*colors];
n_colors = length(colors);

cluster_figures = length(vars_filt)/3;
cluster_figures = [7 4 3 4];

cumsum_cluster_figures = [0 cumsum([cluster_figures])];
for ff=1:length(cluster_figures)
  figure(ff)
  plot_vars = cumsum_cluster_figures(ff)+1:cumsum_cluster_figures(ff+1);
  
  for d=1:3
    subplot(3,1,d)
    labels = {};
    % Input data
    for i=1:length(plot_vars)
      index = indices{d}(plot_vars(i));
      plot(time_in,D_in(:,index),'Color',1.0*colors(mod(i,n_colors)+1,:),'LineWidth',4)
      hold on
      labels{end+1} = strrep(vars_in(index).name, '_', '-');
    end
    % Cropped and filtered data
    for i=1:length(plot_vars)
      index = (plot_vars(i)-1)*3+d;
      plot(time_out,D_filt(:,index),'Color',0.4*colors(mod(i,n_colors)+1,:),'LineWidth',2)
      ylabel(vars_filt(index).unit)
    end
    if (d>1)
      plot([t_0 t_end],[0 0],'-ok');
    end
    hold off
    legend(labels,'Location','EastOutside')
    xlabel('s')
  end
end

%for ii=1:length(vars_filt)
%  fprintf('%d %s %s\n',ii,vars_filt(ii).name,vars_filt(ii).unit);
%end

end


%%-----------------------------------------------------------------------------
% Freek: The following 3 functions were sent to me by Peter in 2008. They work, so I haven't touched them since then ;-)
function [ out_Y_target, out_Y_scaled ] = scale_filt(raw_position_data,trajectory_size,fc_factor)

traj_size_pos_vel_acc = 3;

n_ts = length(raw_position_data(2,:)) - 1; %time_stamp

num_of_samples = length(raw_position_data(:,1));

duration = diff(raw_position_data([1 end],1));

Y_target = zeros(num_of_samples,n_ts*traj_size_pos_vel_acc);

for i=1:n_ts,
    Y_target(:,(i-1)*3+1) = raw_position_data(:,i+1);
end

%end_1 = length(raw_position_data(:,n_ts+1)); % n_ts+1 -> first row is time
%duration = raw_position_data(end_1,n_ts+1) - raw_position_data(1,n_ts+1);
%duration = raw_position_data(length(raw_position_data),1) - raw_position_data(1,1);

if(duration <= 0)
   fprintf('ERROR: duration is %f s !!!',duration); 
   keyboard;
end

for x_i=0:n_ts-1
    
    % add 2 points since they will be used to calculate the derivatives
    target_trajectory_scaled_position(:,x_i+1) = scale_up(Y_target(:,x_i*traj_size_pos_vel_acc+1),trajectory_size+2);
    target_trajectory(:,x_i*traj_size_pos_vel_acc+1:x_i*traj_size_pos_vel_acc+traj_size_pos_vel_acc) = filt_and_diff(target_trajectory_scaled_position(:,x_i+1),duration,fc_factor,trajectory_size);
end

%figure(12)
%hold on;
%plot3(target_trajectory_scaled_position(:,1),target_trajectory_scaled_position(:,2),target_trajectory_scaled_position(:,3)); 
%hold off;

out_Y_target = target_trajectory;
out_Y_scaled = target_trajectory_scaled_position;
end

function [varargout] = scale_up(track,new_trajectory_length)

    cut_off = 3;

    %tmptrack = zeros(3*length(track),1);
    tmptrack = zeros(cut_off*length(track),1);
    
    for i=1:length(tmptrack)
        if(i > ceil(cut_off/2)*length(track))
            tmptrack(i) = track(end);
        elseif(i > floor(cut_off/2)*length(track))
            tmptrack(i) = track(i-floor(cut_off/2)*length(track));
        else
            tmptrack(i) = track(1);
        end;
    end;
    
    %tmptrack2 = zeros(3*length(track),1);
    %for j=1:3
    %    for i=1:length(track)
    %        tmptrack2(i+((j-1)*length(track))) = track(i);
    %    end
    %end
    
    shift = 0;
    
    resampled_full_track = resample(tmptrack,new_trajectory_length,length(track));
    output_track = resampled_full_track((floor(cut_off/2)*new_trajectory_length)+1+shift:(ceil(cut_off/2)*new_trajectory_length)+shift);

%    figure(232)
%    hold on;
%    plot(track,'g')
%    plot(resampled_full_track,'m')
%    plot(tmptrack,'b');
%    plot(output_track,'c')
%    hold off;
    
    varargout(1) = {output_track};
end

function [varargout] = filt_and_diff(Y_target,duration,fc_factor,trajectory_size)
       
       % calculating the coefficients (B,A) for the filtfilt function to
       % smooth the trajectory... TODO: tune the parameters fc, fs and order.       
       fs = length(Y_target(:,1))/duration;
       %fc = fs * fc_factor / length(Y_target(:,1));
       fc = fc_factor / duration;
       % zzz For me, order=2 works better...
       order = 5;
       % disp(sprintf('filt_and_diff>> duration=%f ... fc = %f fs = %f order = %i',duration,fc,fs,order));
       [B,A] = butter(order,2*fc/fs);

       % filt position
       yt = zeros(length(Y_target(:,1)),1);
       yt = filtfilt(B,A,Y_target(:,1));
       Y_target(:,1) = yt;

       % use different frequenzy cut (fc) to filt the velocity and acceleration
       %fc = fs * 5 / length(Y_target(:,1));
       fc = 5 / duration;
       [B,A] = butter(order,2*fc/fs);

       filtered_yt = zeros(length(Y_target(:,1)),1);
       filtered_yt = filtfilt(B,A,Y_target(:,1));

       yt_d = zeros(length(Y_target(:,1)),1);
       %yt_d = [0;diff(filtered_yt) ./ (duration/length(filtered_yt))];
       %yt_d = [0;diff(yt) ./ (duration/length(yt))];
       yt_d = [diff(yt) ./ (duration/length(yt));0];
       
       clamp_to_zero = 0;
       if (clamp_to_zero)
        myfilt = linspace(yt_d(1),yt_d(end-1),length(yt_d))'  ;
        yt_d = yt_d-myfilt;
       end
       
       Y_target(:,2) = yt_d;

       %TODO CHECK
       filtered_yt_d = zeros(length(Y_target(:,1)),1);
       % filtered_yt_d = filtfilt(B,A,Y_target(:,1));
       filtered_yt_d = filtfilt(B,A,Y_target(:,2));

       yt_dd = zeros(length(Y_target(:,1)),1);
       %yt_dd = [0;diff(filtered_yt_d) ./ (duration/length(filtered_yt_d))];
       %yt_dd = [0;diff(yt_d) ./ (duration/length(yt_d))];
       yt_dd = [diff(yt_d) ./ (duration/length(yt_d));0];

       if (clamp_to_zero)
         n = length(yt_dd);
         gs = gaussian(n,20/n);
         cumsumgs = cumsum(gs)/max(cumsum(gs));
         cumsumgs = cumsumgs.*cumsumgs;
         first = find(cumsumgs>0.01,1);
         final = find(cumsumgs>0.99,1);
         myfilt = [cumsumgs(first:final); ones(n-2*(final-first+1),1);  cumsumgs(final:-1:first)];
         yt_dd = yt_dd.*myfilt;
       end

       Y_target(:,3) = yt_dd;

       Y_target = Y_target(1:trajectory_size,:);
       varargout(1) = {Y_target};
end