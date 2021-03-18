1) Prepare to gather the right variable for the right amount of time, by calling:

  cd ~/prog/armUser
  addpath matlab/kinestheticteaching
  get_kinesthetic_teaching_var_names('prefs/task_default.script')

This saves the pos/vel/acc names of the joints, as well as the end-effector position and orientation to the 'task_default.script' file. This makes sure all the relevant variables are written to an mrd file.

Also, set the amount of time you want to collect data by editing 'task_default_sampling_time_sec' in 'prefs/default_script'. The relevant data will be cropped later, so rather set this higher than you think you need (Hofstaedter's law: Things will always take longer than you think, even if you take into account Hofstaedter's law)

2) Gather data on the robot with the 'Traj task' (or ask Ludo what the best way to do this is)

3) Post-process data with: 

  process_kinesthetic_teaching(t_range,duration_closing,inputfilename,outputfilename)

t_range is the time you select; the rest will be cropped. Because the fingers cannot be moved (yet) through kinesthesis this script also appends data to the mrd plot that moves the hand from an open posture (which it maintains during the motion of the end-effector) to a closed posture. The amount of time that is taken for this is set with 'duration_closing'. The inputfilename and outputfilename can be set. The default is to read from the file specifified in '.last_data'. For example:
  
  process_kinesthetic_teaching([1 7], 4, 'recorded.mrd', 'demonstration.mrd')

  
append_close_gripper.m
crop_and_filter_mrd.m
fixquaternionsign.m
get_joint_names.m
get_kinesthetic_teaching_var_names.m
min_jerk.m
process_kinesthetic_teaching.m

