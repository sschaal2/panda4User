function [joint_names] = get_joint_names();
% Get the joint names of the robot, by reading them from config/WhichDOFs.cf

% Read file
fid = fopen('config/WhichDOFs.cf');
words = textscan(fid,'%s');
words = words{1};
fclose(fid);

% Find the last 'task_servo' label
task_servo_word_index = find(strcmp(words,'task_servo'),1,'last');

% Count up to the next '*_servo' label
% Between those two labels lie the joint_names
joint_names = {};
cur_index = task_servo_word_index+1;
while (isempty(strfind(words{cur_index},'servo')))
  word = words(cur_index);
  joint_names{end+1} = word{:};
  cur_index = cur_index+1;
end



end