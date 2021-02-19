function process_kinesthetic_teaching(t_range,duration_closing,inputfilename,outputfilename)
% Process the result of gathering data with kinesthetic teaching.
%
% It crops the data, filters it with a Butterworth filter, and appends data for
% closing the gripper. The variables names in the output mrd files are always
% desired values (i.e. R_SFE_des_th)
%
% Parameters:
%   t_range           : relevant range in seconds; rest is cropped away
%                           (default: all data)
%   duration_closing  : number of seconds in which the gripper should close
%                           (default: 3.0s)
%   inputfilename     : input mrd file (default read from .lastdata)
%   outputfilename    : output mrd file

delete_intermediate_mrd_files = 0;

%-------------------------------------------------------------------------------
% PROCESS ARGUMENTS: SET DEFAULTS IF NECESSARY

if (~exist('t_range','var')), t_range = []; end
if (~exist('duration_closing','var')), duration_closing = 3.0; end

% Determine input filename
if (~exist('inputfilename','var') || isempty(inputfilename))
  if exist('.last_data','file')
    inputfilename = sprintf('d%05d',load('.last_data')-1);
  else
    fprintf('ERROR: You must provide a filename.\n');
    return;
  end
end

% Determine output filename
if (~exist('outputfilename','var') || isempty(outputfilename))
  outputfilename = strrep([inputfilename '.mrd' ],'.mrd.mrd','.mrd');
  outputfilename = strrep(outputfilename,'.mrd','_taught.mrd');
end

%-------------------------------------------------------------------------------
% VARIABLE NAMES THAT ARE PROCESSED
[var_names var_des_names] = get_kinesthetic_teaching_var_names;

%-------------------------------------------------------------------------------
% READ INPUT FILE
fprintf('Reading from file "%s"\n',inputfilename);
addpath ~clmc/matlab/utilities/
[D, vars, frequency] = clmcplot_convert(inputfilename);

% Collect variable names in the mrd file in a cell array of strings
input_var_names = cell(1,length(vars));
for vv=1:length(vars)
  input_var_names{vv} = vars(vv).name; 
end

% Make sure all variable names exist in the mrd file
for vv=1:length(var_names)
  % Get index of the variable we are looking for
  index = find(strcmp(var_names{vv},input_var_names));
  if (isempty(index))
    fprintf('ERROR: Could not find variable "%s" in file "%s"\n',var_names{vv},inputfilename);
    return;
  end
end


%-------------------------------------------------------------------------------
% EXTRACT OBSERVED VARIABLES, AND SAVE AS DESIRED
fprintf('CONVERT OBSERVED INTO DESIRED TRAJECTORIES\n');
D_output = zeros(size(D,1),length(var_names));
for vv=1:length(var_names)
  index = find(strcmp(var_names{vv},input_var_names));
  D_output(:,vv)       = D(:,index);
  vars_output(vv).name = var_des_names{vv};
  vars_output(vv).unit = vars(index).unit;  
end

desiredfilename = [ strrep(inputfilename,'.mrd','') '_desired.mrd' ];
fprintf('Saving to file "%s"\n',desiredfilename);
clmcplot_gen(D_output,vars_output,frequency,desiredfilename)

%-------------------------------------------------------------------------------
% CROP AND FILTER FILE
fprintf('CROP AND FILTER FILE\n');
[croppedfilename] = crop_and_filter_mrd(t_range,desiredfilename);

%-------------------------------------------------------------------------------
% CLOSE GRIPPER AT THE END
fprintf('CLOSE GRIPPER AT THE END\n');
[closingfilename] = append_close_gripper(croppedfilename,duration_closing);
% Rename to output file
system(['cp ' closingfilename ' ' outputfilename]);
fprintf('Saved to %s\n',outputfilename);

%-------------------------------------------------------------------------------
% REMOVE INTERMEDIATE FILES
if (delete_intermediate_mrd_files)
  fprintf('REMOVE INTERMEDIATE FILES\n');
  system(['rm ' closingfilename ' ' croppedfilename ' ' desiredfilename ' ' ]);
end


end