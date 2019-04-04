/*============================================================================
==============================================================================
                      
                              qfsp_task.c
 
==============================================================================
Remarks:

      a first shot at a qfsp inertion taks

============================================================================*/

// general includes
#include "SL_system_headers.h"

// private includes
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_common.h"

// defines
enum StateMachineStates
  {
   INIT_SM_TARGET,
   GRIPPER_START,
   GRIPPER_END,
   MOVE_TO_TARGET,
   FINISH_SM_TARGET,
   IDLE
  };

enum Controllers
  {
   SIMPLE_IMPEDANCE_JT=1,
   N_CONT
  };

static int n_controllers = N_CONT-1;

char controller_names[][100]=
  {
   {"dummy"},
   {"SimpleImpedanceJt"}
  };


// variables for filtering
#define FILTER_ORDER 2
#define N_FILTERS    50

static float filters_a[N_FILTERS+1][FILTER_ORDER+1];
static float filters_b[N_FILTERS+1][FILTER_ORDER+1];

typedef struct Filter {
  int cutoff;
  double raw[3];
  double filt[3];
} Filter;

static Filter fthdd[N_DOFS+1];


typedef struct StateMachineTarget {
  char   state_name[100];
  double pose_x[N_CART+1];
  int    use_orient;
  double pose_q[N_QUAT+1];
  double movement_duration;
  double gripper_width_start;
  double gripper_width_end;
  double gripper_force_start;
  double gripper_force_end;
  double cart_force_start;
  double cart_force_end;
  double cart_moment_start;
  double cart_moment_end;
  int    use_default_gains;
  double cart_gain_x[N_CART+1]; // diagonal of matrix
  double cart_gain_xd[N_CART+1]; // diagonal of matrix
  double cart_gain_a[N_CART+1];  // diagonal of matrix
  double cart_gain_ad[N_CART+1]; // diagonal of matrix
  char   controller_name[100];
} StateMachineTarget;

#define MAX_STATES_SM 1000
static StateMachineTarget targets_sm[MAX_STATES_SM+1];
static int n_states_sm = 0;
static int current_state_sm = 0;


/* Cartesian orientation representation with a rotation around an axis */
typedef struct { 
  double phi;          /* rotation angle */
  double r[N_CART+1];  /* rotation axis  */
} My_Crot;

/* local variables */
static double     time_step;
static double     cref[N_ENDEFFS*6+1];
static SL_Cstate  ctarget[N_ENDEFFS+1];
static SL_Cstate  cdes[N_ENDEFFS+1];
static SL_quat    ctarget_orient[N_ENDEFFS+1];
static My_Crot    ctarget_rot[N_ENDEFFS+1];  
static SL_quat    cdes_orient[N_ENDEFFS+1];
static SL_quat    cdes_start_orient[N_ENDEFFS+1]; 
static double     corient_error[N_ENDEFFS*3+1];
static int        stats[N_ENDEFFS*6+1];
static SL_DJstate target[N_DOFS+1];
static SL_DJstate last_target[N_DOFS+1];
static int        firsttime = TRUE;
static double     start_time     = 0;
static double     default_gain   = 450;
static double     default_gain_orient = 40;

static SL_Cstate  ball_state;

static double controller_gain_th[N_DOFS+1];
static double controller_gain_thd[N_DOFS+1];
static double controller_gain_int[N_DOFS+1];

static int    state_machine_state = INIT_SM_TARGET;

static double s[3+1]; // indicator for min jerk in orientation space

/* global functions */
void add_qfsp_task(void);

/* local functions */
static int    cartesianImpedanceSimpleJt(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
					 Vector cart, iVector status, double dt, double t);
static int    init_qfsp_task(void);
static int    run_qfsp_task(void);
static int    change_qfsp_task(void);
static int    init_filters(void);
static double filt(double raw, Filter *fptr);
static void   quatDerivativesToAngVelAcc(SL_quat *q);
static int    teach_target_pose(void);
static Matrix SL_inertiaMatrix(SL_Jstate *lstate, SL_Cstate *cbase, 
			       SL_quat *obase, SL_endeff *leff);
static int    min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
				  double t_togo, double dt,
				  double *x_next, double *xd_next, double *xdd_next);
static int    min_jerk_next_step_quat (SL_quat q_current, SL_quat q_target, double *s,
				       double t_togo, double dt, SL_quat *q_next);
static int    read_state_machine(char *fname);
static void   print_sm_state(void);


/*****************************************************************************
******************************************************************************
Function Name	: add_qfsp_task
Date		: Feb 2019
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_qfsp_task( void )
{
  int i, j;
  char string[100];
  
  addTask("QFSP Insertion Task", init_qfsp_task, 
	  run_qfsp_task, change_qfsp_task);

  addToMan("print_sm_state","prints the current state suitable for state machine",print_sm_state); 

  /* read the control gains  */
  if (!read_gains(config_files[GAINS],controller_gain_th, 
		  controller_gain_thd, controller_gain_int))
    return;
  
  
}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_qfsp_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_qfsp_task(void)
{
  int    j, i;
  char   string[100];
  char   fname[100] = "qfsp.sm";
  int    ans;
  int    flag = FALSE;
  static int firsttime = TRUE;
  
  if (firsttime) {
    
    // initialize the filters 
    firsttime = FALSE;
    init_filters();
    for (i=1; i<=N_DOFS; ++i) 
      fthdd[i].cutoff = 5;

    // zero out variables
    bzero((char *)&cref,sizeof(cref));
    bzero((char *)&ctarget,sizeof(ctarget));
    bzero((char *)&ctarget_orient,sizeof(ctarget_orient));


    // add variables to data collection
    for (i=1; i<=N_ENDEFFS; ++i) {
      sprintf(string,"%s_ct_x",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].x[_X_]),string,"m", DOUBLE,FALSE);
      sprintf(string,"%s_ct_y",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].x[_Y_]),string,"m", DOUBLE,FALSE);
      sprintf(string,"%s_ct_z",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].x[_Z_]),string,"m", DOUBLE,FALSE);
      
      sprintf(string,"%s_ct_xd",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].xd[_X_]),string,"m/s", DOUBLE,FALSE);
      sprintf(string,"%s_ct_yd",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].xd[_Y_]),string,"m/s", DOUBLE,FALSE);
      sprintf(string,"%s_ct_zd",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].xd[_Z_]),string,"m/s", DOUBLE,FALSE);

      sprintf(string,"%s_ct_xdd",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].xdd[_X_]),string,"m/s^2", DOUBLE,FALSE);
      sprintf(string,"%s_ct_ydd",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].xdd[_Y_]),string,"m/s^2", DOUBLE,FALSE);
      sprintf(string,"%s_ct_zdd",cart_names[i]);
      addVarToCollect((char *)&(ctarget[i].xdd[_Z_]),string,"m/s^2", DOUBLE,FALSE);

      /* orientation variables */
      sprintf(string,"%s_ct_q0",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].q[_Q0_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q1",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].q[_Q1_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q2",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].q[_Q2_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q3",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].q[_Q3_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_q0d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qd[_Q0_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q1d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qd[_Q1_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q2d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qd[_Q2_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q3d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qd[_Q3_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_q0dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qdd[_Q0_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q1dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qdd[_Q1_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q2dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qdd[_Q2_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q3dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].qdd[_Q3_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_ad",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].ad[_A_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_bd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].ad[_B_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_gd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].ad[_G_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_add",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].add[_A_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_bdd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].add[_B_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_gdd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_orient[i].add[_G_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_corient_e1",cart_names[i]);
      addVarToCollect((char *)&(corient_error[(i-1)*3 + _A_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_corient_e2",cart_names[i]);
      addVarToCollect((char *)&(corient_error[(i-1)*3 + _B_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_corient_e3",cart_names[i]);
      addVarToCollect((char *)&(corient_error[(i-1)*3 + _G_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_cart_ref_x",cart_names[i]);
      addVarToCollect((char *)&(cref[(i-1)*6 + 1]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_y",cart_names[i]);
      addVarToCollect((char *)&(cref[(i-1)*6 + 2]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_z",cart_names[i]);
      addVarToCollect((char *)&(cref[(i-1)*6 + 3]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_a",cart_names[i]);
      addVarToCollect((char *)&(cref[(i-1)*6 + 4]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_b",cart_names[i]);
      addVarToCollect((char *)&(cref[(i-1)*6 + 5]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_g",cart_names[i]);
      addVarToCollect((char *)&(cref[(i-1)*6 + 6]),string,"-", DOUBLE,FALSE);
      
      sprintf(string,"%s_cdes_x",cart_names[i]);
      addVarToCollect((char *)&(cdes[i].x[_X_]),string,"m", DOUBLE,FALSE);
      sprintf(string,"%s_cdes_y",cart_names[i]);
      addVarToCollect((char *)&(cdes[i].x[_Y_]),string,"m", DOUBLE,FALSE);
      sprintf(string,"%s_cdes_z",cart_names[i]);
      addVarToCollect((char *)&(cdes[i].x[_Z_]),string,"m", DOUBLE,FALSE);
      
      
      
    }
    
    updateDataCollectScript();
    
    
  } else { // not firstime
    
    // zero the filters 
    for (i=1; i<=N_DOFS; ++i) 
      for (j=0; j<=FILTER_ORDER; ++j)
	fthdd[i].raw[j] = fthdd[i].filt[j] = 0;
    
  }
  
  // check whether any other task is running 
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Task can only be started if no other task is running!\n");
    return FALSE;
  }
  
  // read state machine
  ans = TRUE;
  get_int("Read state machine from file?",ans,&ans);
  if (ans) {
    if (get_string("File name of state machine in prefs/",fname,fname)) {
      read_state_machine(fname);
    }
  }
  if (n_states_sm <= 0) {
    printf("No valid state machine found\n");
    return FALSE;
  }
  
  // go to a save posture 
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  bzero((char *)&(last_target[1]),N_DOFS*sizeof(last_target[1]));
  for (i=1; i<=N_DOFS; ++i) {
    target[i] = joint_default_state[i];
    last_target[i] = joint_default_state[i];
  }
  target[J4].th -= 0.2;
  target[J6].th += 0.2;

  sendGripperMoveCommand(0.05,0.1);

  if (!go_target_wait_ID(target))
    return FALSE;


  // initialize the cartesian control
  bzero((char *)&cdes,sizeof(cdes));
  bzero((char *)&cdes_orient,sizeof(cdes_orient));
  
  for (j=1; j<=N_CART; ++j) {
    
    // by default, we assume full pose control
    stats[j] = 1;
    stats[j + N_CART] = 1;
    
    // set cartesian target and current desired to current state to be safe
    cdes[HAND].x[j] = ctarget[HAND].x[j] = cart_des_state[HAND].x[j];
  }
  
  for (j=1; j<=N_QUAT; ++j) {
    cdes_orient[HAND].q[j] = ctarget_orient[HAND].q[j] = cart_des_orient[HAND].q[j];
  }

  // reclibrate the gripper offsets
  sendCalibrateFTCommand();

  
  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  if (ans != 1) 
    return FALSE;


  time_step = 1./(double)task_servo_rate;
  start_time = task_servo_time;
  state_machine_state = INIT_SM_TARGET;
  current_state_sm = 0;
  
  scd();
  
  
  return TRUE;
  
  }
  
/*****************************************************************************
******************************************************************************
  Function Name	: run_qfsp_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_qfsp_task(void)
{
  int j, i;
  double sum=0;
  double dist;
  double aux;
  static double time_to_go;
  float pos[N_CART+1];
  double gripper_move_threshold = 1e-8;
  static int wait_ticks=0;
  int    no_gripper_motion = FALSE;


  switch (state_machine_state) {

  case INIT_SM_TARGET:
    
    // check whether to end state machine
    if (current_state_sm < n_states_sm) {
      ++current_state_sm;
    } else {
      freeze();
      return TRUE;
    }
    
    // assign relevant variables from state machine state array
    time_to_go = targets_sm[current_state_sm].movement_duration;
    
    for (i=1; i<=N_CART; ++i)
      ctarget[HAND].x[i] = targets_sm[current_state_sm].pose_x[i];
    
    if (targets_sm[current_state_sm].use_orient) {
      
      for (i=1; i<=N_CART; ++i)
	stats[N_CART+i] = 1;
      
      for (i=1; i<=N_QUAT; ++i)
	ctarget_orient[HAND].q[i] = targets_sm[current_state_sm].pose_q[i];
      
    } else { // no orientation
      
      for (i=1; i<=N_CART; ++i)
	stats[N_CART+i] = 0;
      
    }
    
    // need to memorize the cart orient start for min jerk interpolation
    cdes_start_orient[HAND] = cdes_orient[HAND];
    
    // gripper movement: only if gripper states have changed
    no_gripper_motion = FALSE;
    if (current_state_sm > 1) {
      if (targets_sm[current_state_sm].gripper_width_start == targets_sm[current_state_sm-1].gripper_width_end &&
	  targets_sm[current_state_sm].gripper_force_start == targets_sm[current_state_sm-1].gripper_force_end) {
	no_gripper_motion=TRUE; 
      }
    }

    if (!no_gripper_motion) {
      // give move command to gripper to desired position if width is larger than current width
      if (targets_sm[current_state_sm].gripper_width_start > misc_sensor[G_WIDTH]) {
	sendGripperMoveCommand(targets_sm[current_state_sm].gripper_width_start,0.1);
      } else { // or close gripper with force control otherwise
	sendGripperGraspCommand(targets_sm[current_state_sm].gripper_width_start,
				0.1,
				targets_sm[current_state_sm].gripper_force_start,
				0.08,
				0.08);
      }
      wait_ticks = 100; // need to give non-real-time gripper thread a moment to get started
      state_machine_state = GRIPPER_START;
    } else {
      state_machine_state = MOVE_TO_TARGET;
    }
    
    // prepare min jerk for orientation space: s is an interpolation variable 
    s[1] = s[2] = s[3] = 0;
    
    break;
    
    
  case GRIPPER_START:
    
    if (--wait_ticks < 0) {
      if (misc_sensor[G_MOTION] == 0) {
	state_machine_state = MOVE_TO_TARGET;
      }
    }
    
    break;
    
    
  case MOVE_TO_TARGET:
    
    // plan the next step of hand with min jerk
    for (i=1; i<=N_CART; ++i) {
      min_jerk_next_step(cdes[HAND].x[i],
			 cdes[HAND].xd[i],
			 cdes[HAND].xdd[i],
			 ctarget[HAND].x[i],
			 ctarget[HAND].xd[i],
			 ctarget[HAND].xdd[i],
			 time_to_go,
			 time_step,
			 &(cdes[HAND].x[i]),
			 &(cdes[HAND].xd[i]),
			 &(cdes[HAND].xdd[i]));
    }
    
    if (targets_sm[current_state_sm].use_orient) {
      min_jerk_next_step_quat(cdes_start_orient[HAND], ctarget_orient[HAND], s,
			      time_to_go, time_step, &(cdes_orient[HAND]));
    }
    
    time_to_go -= time_step;
    if (time_to_go < 0) {
      time_to_go = 0;

      no_gripper_motion = FALSE;
      if (targets_sm[current_state_sm].gripper_width_end == targets_sm[current_state_sm].gripper_width_start &&
	  targets_sm[current_state_sm].gripper_force_end == targets_sm[current_state_sm].gripper_force_start) {
	no_gripper_motion=TRUE; 
      }

      if (!no_gripper_motion) {
	if (targets_sm[current_state_sm].gripper_width_end > misc_sensor[G_WIDTH]) {
	  sendGripperMoveCommand(targets_sm[current_state_sm].gripper_width_end,0.1);
	} else {
	  sendGripperGraspCommand(targets_sm[current_state_sm].gripper_width_end,
				  0.1,
				  targets_sm[current_state_sm].gripper_force_end,
				  0.08,
				  0.08);
	}
	wait_ticks = 100; // need to give non-real-time gripper thread a moment to get started
	state_machine_state = GRIPPER_END;
      } else {
	state_machine_state = INIT_SM_TARGET;
      }
    }

    break;

  case GRIPPER_END:
    if (--wait_ticks < 0) {
      if (misc_sensor[G_MOTION] == 0) {
	state_machine_state = INIT_SM_TARGET;
      }
    }

    break;

  case IDLE:
    break;

  default:
    break;

  }
  

  // compute orientation error term for quaterion feedback control
  quatErrorVector(cdes_orient[1].q,cart_orient[1].q,corient_error);
  
  // prepare the impdance controller, i.e., compute operational
  // space force command

  if (targets_sm[current_state_sm].use_default_gains) {

    for (j= _X_; j<= _Z_; ++j) {
      if (stats[j]) {
	cref[j] =
	  (cdes[HAND].xd[j]  - cart_state[HAND].xd[j]) * 2.*sqrt(default_gain) +
	  (cdes[HAND].x[j]  - cart_state[HAND].x[j]) * default_gain;
      }
    }
    
    for (j= _A_; j<= _G_ ; ++j) { /* orientation */
      if (stats[N_CART + j]) {
	cref[N_CART + j] = 
	  (cdes_orient[HAND].ad[j] - cart_orient[HAND].ad[j]) *0.025 * 2.0 * sqrt(default_gain_orient) - 
	  corient_error[j] * default_gain_orient; 
      }
    }

  } else { // use gains from targets_sm

    for (j= _X_; j<= _Z_; ++j) {
      if (stats[j]) {
	cref[j] =
	  (cdes[HAND].xd[j]  - cart_state[HAND].xd[j]) *  targets_sm[current_state_sm].cart_gain_xd[j] +
	  (cdes[HAND].x[j]  - cart_state[HAND].x[j]) * targets_sm[current_state_sm].cart_gain_x[j];
      }
    }
    
    for (j= _A_; j<= _G_ ; ++j) { /* orientation */
      if (stats[N_CART + j]) {
	cref[N_CART + j] = 
	  (cdes_orient[HAND].ad[j] - cart_orient[HAND].ad[j]) * targets_sm[current_state_sm].cart_gain_ad[j] - 
	  corient_error[j] * targets_sm[current_state_sm].cart_gain_a[j]; 
      }
    }

  }

  // the inverse kinematics controller needs the current state as input;
  // the is no notion of a desire joint state, i.e., it is equal to the
  // current state as only the operational space servo is active
  bzero((char *)&target,sizeof(target));
  for (i=1; i<=N_DOFS; ++i) {
    target[i].th  = joint_state[i].th;
    target[i].thd = joint_state[i].thd;    
  }

  // this computes the joint space torque
  if (!cartesianImpedanceSimpleJt(target,endeff,joint_opt_state,
				  cref,stats,time_step,task_servo_time)) {
    freeze();
    return FALSE;
  }

  for (i=1; i<=N_DOFS; ++i) {
    joint_des_state[i].thdd  = 0.0;
    joint_des_state[i].thd   = joint_state[i].thd;
    joint_des_state[i].th    = joint_state[i].th;
    joint_des_state[i].uff   = 0.0;
  }
    
    // inverse dynamics
  SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

  // add feedforward torques from impedance controller
  for (i=1; i<=N_DOFS; ++i) {
    joint_des_state[i].uff   += target[i].uff;
  }

  // visualize the cartesian desired as a ball
  for (i=1; i<=N_CART; ++i)
    pos[i] = cdes[HAND].x[i];
  sendUserGraphics("ball",&(pos[_X_]), N_CART*sizeof(float));


  return TRUE;
  
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_qfsp_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_qfsp_task(void)
{
  int j, i;
  char string[100];
  double aux;
  double aux_r[N_CART+1];
  double aux_r_norm=0;
  double angle;

  for (i=_Q1_; i<=_Q3_; ++i) 
    aux_r_norm += ctarget_orient[HAND].q[i];

  for (i=_Q1_; i<=_Q3_; ++i) 
    aux_r[i-1] = ctarget_orient[HAND].q[i]/aux_r_norm;

  angle = acos(ctarget_orient[HAND].q[_Q0_])/PI*180.*2.;


  do{
    get_double("Target Cart rot X axis",aux_r[_A_], &aux_r[_A_]);
    get_double("Target Cart rot Y axis",aux_r[_B_], &aux_r[_B_]);
    get_double("Target Cart rot Z axis",aux_r[_G_], &aux_r[_G_]);
    
    aux_r_norm = sqrt(aux_r[_A_]*aux_r[_A_] + aux_r[_B_]*aux_r[_B_]+ aux_r[_G_]*aux_r[_G_]);
  } while (aux_r_norm < 0.001);

  for (i=1; i<=N_CART; ++i)
    aux_r[i] /= aux_r_norm;

  do{
    get_double("Target Cart rot angle phi (-180~180 deg)",angle, &angle);
  } while (angle < -180 || angle > 180);
    
  angle = angle * PI/180.0;

  ctarget_orient[HAND].q[_Q0_] = cos(angle/2.0);
  ctarget_orient[HAND].q[_Q1_] = aux_r[_A_]*sin(angle/2.0);
  ctarget_orient[HAND].q[_Q2_] = aux_r[_B_]*sin(angle/2.0);
  ctarget_orient[HAND].q[_Q3_] = aux_r[_G_]*sin(angle/2.0);

  printf("Target cart quaternion = [%.3f, %.3f, %.3f, %.3f]\n",
	 ctarget_orient[HAND].q[_Q0_],
	 ctarget_orient[HAND].q[_Q1_],
	 ctarget_orient[HAND].q[_Q2_],
	 ctarget_orient[HAND].q[_Q3_]);
  

  return TRUE;

}

/*****************************************************************************
******************************************************************************
  Function Name	: teach_target_pose
  Date		: March 2019

  Remarks:

  extracts target pose from current endeffector pose

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
teach_target_pose(void)
{
  int    i,j;
  double aux_r[N_CART+1];
  double aux_x[N_CART+1];
  double aux_r_norm=0;
  double aux;
  double angle;

  printf("Target position = [%.3f, %.3f, %.3f]\n",
	 ctarget[HAND].x[_X_],
	 ctarget[HAND].x[_Y_],
	 ctarget[HAND].x[_Z_]);

  printf("Target cart quaternion = [%.3f, %.3f, %.3f, %.3f]\n",
	 ctarget_orient[HAND].q[_Q0_],
	 ctarget_orient[HAND].q[_Q1_],
	 ctarget_orient[HAND].q[_Q2_],
	 ctarget_orient[HAND].q[_Q3_]);

  // extract pose from current position
  for (i=_Q1_; i<=_Q3_; ++i) 
    aux_r_norm += cart_orient[HAND].q[i];

  for (i=_Q1_; i<=_Q3_; ++i) 
    aux_r[i-1] = cart_orient[HAND].q[i]/aux_r_norm;

  // angle in degrees
  angle = acos(cart_orient[HAND].q[_Q0_])/PI*180.*2;

  for (i=1; i<=N_CART; ++i)
    aux_x[i] = cart_state[HAND].x[i];

  // offer modify of pose
  get_double("Target Cart X position",aux_x[_X_], &aux_x[_X_]);
  get_double("Target Cart Y position",aux_x[_Y_], &aux_x[_Y_]);
  get_double("Target Cart Z position",aux_x[_Z_], &aux_x[_Z_]);


  do{
    get_double("Target Cart rot X axis",aux_r[_A_], &aux_r[_A_]);
    get_double("Target Cart rot Y axis",aux_r[_B_], &aux_r[_B_]);
    get_double("Target Cart rot Z axis",aux_r[_G_], &aux_r[_G_]);
    
    aux_r_norm = sqrt(aux_r[_A_]*aux_r[_A_] + aux_r[_B_]*aux_r[_B_]+ aux_r[_G_]*aux_r[_G_]);
  } while (aux_r_norm < 0.001);

  for (i=1; i<=N_CART; ++i)
    aux_r[i] /= aux_r_norm;

  do{
    get_double("Target Cart rot angle phi (-180~180 deg)",angle, &angle);
  } while (angle < -180 || angle > 180);

  // angle in radians
  angle = angle * PI/180.0;

  ctarget[HAND].x[_X_] = aux_x[_X_];
  ctarget[HAND].x[_Y_] = aux_x[_Y_];
  ctarget[HAND].x[_Z_] = aux_x[_Z_];

  ctarget_orient[HAND].q[_Q0_] = cos(angle/2.0);
  ctarget_orient[HAND].q[_Q1_] = aux_r[_A_]*sin(angle/2.0);
  ctarget_orient[HAND].q[_Q2_] = aux_r[_B_]*sin(angle/2.0);
  ctarget_orient[HAND].q[_Q3_] = aux_r[_G_]*sin(angle/2.0);

  printf("Target position = [%.3f, %.3f, %.3f]\n",
	 ctarget[HAND].x[_X_],
	 ctarget[HAND].x[_Y_],
	 ctarget[HAND].x[_Z_]);

  printf("Target cart quaternion = [%.3f, %.3f, %.3f, %.3f]\n",
	 ctarget_orient[HAND].q[_Q0_],
	 ctarget_orient[HAND].q[_Q1_],
	 ctarget_orient[HAND].q[_Q2_],
	 ctarget_orient[HAND].q[_Q3_]);
 
  
  return TRUE;

}

/*****************************************************************************
******************************************************************************
Function Name	: cartesianImpedanceSimpleJt
Date		: March 2019
   
Remarks:

        computes a very simple Jacobian transpose impedance controller without
        any dynamics model
        
******************************************************************************
Paramters:  (i/o = input/output)

     state   (i/o): the state of the robot (given as a desired state; note
                    that this desired state coincides with the true state of
                    the robot, as the joint space PD controller is replaced
                    by the Cartesian controller)
     endeff  (i)  : the endeffector parameters
     rest    (i)  : the optimization posture
     cart    (i)  : the desired cartesian accelerations (trans & rot)
     status  (i)  : which rows to use from the Jacobian
     dt      (i)  : the integration time step     
     t       (i)  : the current time -- important for safe numerical 
                    differntiation of the Jacobian

 the function updates the state by adding the appropriate feedforward torques

*****************************************************************************/
static int
cartesianImpedanceSimpleJt(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			   Vector cart, iVector status, double dt, double t)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B;
  static Matrix  M, invM;
  static iVector ind;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static double  last_t;       
  static Vector  e, en;
  double         ridge = 1.e-8;

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    dJdtthd = my_vector(1,6*N_ENDEFFS);
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    invM   = my_matrix(1,N_DOFS,1,N_DOFS);
  }

  /* how many contrained cartesian DOFs do we have? */
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  /* build the pseudo-inverse according to the status information */
  mat_zero(P);
  for (i=1; i<=count; ++i) {
    for (j=i; j<=count; ++j) {
      for (n=1; n<=N_DOFS; ++n) {
	P[i][j] += J[ind[i]][n] * J[ind[j]][n];
      }
      if (i==j) 
	P[i][j] += ridge;
      else
	P[j][i] = P[i][j];
    }
  }

  /* invert the matrix */
  if (!my_inv_ludcmp(P, count, P)) {
    return FALSE;
  }

  /* build the B matrix, i.e., the pseudo-inverse */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=count; ++j) {
      B[i][j]=0.0;
      for (n=1; n<=count; ++n) {
	B[i][j] += J[ind[n]][i] * P[n][j];
      }
    }
  }

  /* the simple cartesion impedance controller only uses J-trans */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff = 0;
    for (j=1; j<=count; ++j) {
      state[i].uff += J[ind[j]][i] * cref[ind[j]];
    }
  }

  /**********************/
  /* the null space term */
  /**********************/

  /* compute the NULL space projection matrix; note that it is computationally
     inefficient to do this, since this is O(d^2), while all we really need is
     some matrix-vector multiplications, which are much cheaper */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=i; j<=N_DOFS; ++j) {

      if (i==j) 
	O[i][j] = 1.0;
      else
	O[i][j] = 0.0;

      for (n=1; n<=count; ++n) {
	O[i][j] -= B[i][n] * J[ind[n]][j];
      }

      if (i!=j)
	O[j][i] = O[i][j];
    }
  }

  /* compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS; ++i) {
    double fac=0.1;
    e[i] = 
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd;
  }
  mat_vec_mult(O,e,en);
  //print_mat("O",O);
  //getchar();

  /* add this as a PD command to uff */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff += en[i];
  }

  return TRUE;

}


/*****************************************************************************
******************************************************************************
Function Name	: filt
Date		: Feb 1999
   
Remarks:

    applies a 2nd order butteworth filter

******************************************************************************
Paramters:  (i/o = input/output)

     raw (i): the raw data
     fptr(i): pointer to the filter data structure 

     returns the filtered value

*****************************************************************************/
static double
filt(double raw, Filter *fptr)

{

  int i,j;

  if (fptr->cutoff == 100) {
    return raw;
  }

  fptr->raw[0] = raw;

  fptr->filt[0] = 
    filters_b[fptr->cutoff][0] * fptr->raw[0] +
    filters_b[fptr->cutoff][1] * fptr->raw[1] + 
    filters_b[fptr->cutoff][2] * fptr->raw[2] -
    filters_a[fptr->cutoff][1] * fptr->filt[1] -
    filters_a[fptr->cutoff][2] * fptr->filt[2];

  fptr->raw[2]  = fptr->raw[1];
  fptr->raw[1]  = fptr->raw[0];
  fptr->filt[2] = fptr->filt[1];
  fptr->filt[1] = fptr->filt[0];

  return fptr->filt[0];

}

/*****************************************************************************
******************************************************************************
Function Name	: init_filters
Date		: Dec 1997
   
Remarks:

        reads the filter coefficients and initializes the appropriate
	variables

******************************************************************************
Paramters:  (i/o = input/output)

     none

*****************************************************************************/
static int
init_filters(void)

{
  
  int i,j,rc;
  FILE *filterfile;
  int count_v=0, count_r=0;
  char string[100];
  
  /* read in the filter file */
  
  sprintf(string,"%sbutterworth_table_order_2",CONFIG);
  filterfile = fopen(string,"r");
  
  if (filterfile == NULL) {
    
    printf("Cannot find filter file >%s<\n",string);
    beep(1); 
    
    return FALSE;
    
  }
  
  for (i=1; i<=N_FILTERS; ++i) {
    ++count_r;
    for (j=0; j<= FILTER_ORDER; ++j) {
      ++count_v;
      rc=fscanf(filterfile,"%f",&(filters_a[i][j]));
    }
    
    for (j=0; j<= FILTER_ORDER; ++j) {
      ++count_v;
      rc=fscanf(filterfile,"%f",&(filters_b[i][j]));
    }
  }
  
  fclose (filterfile);
  
  printf("\nRead file of butterworth filter coefficients: #rows=%d  #val=%d\n",
	 count_r,count_v);
  
  return TRUE;
  
}

/*****************************************************************************
******************************************************************************
Function Name   : quatDerivativesToAngVelAcc
Date            : Jun 2006
Remarks:

 computes the angular velocity from the quaternian derivatives and 
 quaternion

******************************************************************************
Paramters:  (i/o = input/output)

q   (i): structure containing the quaternion, angular velocity and acceleration
q   (o): structure containing the quaternion, angular velocity and acceleration

*****************************************************************************/
static void
quatDerivativesToAngVelAcc(SL_quat *q)
{
  int i,j;
  double  Q[N_CART+1][N_QUAT+1]; 
  double Qd[N_CART+1][N_QUAT+1]; 

  /* initialize Q and Qd */
  for (i=0;i<=N_CART;i++){
    for (j=0;j<=N_QUAT;j++){
      Q[i][j] = 0.0;
      Qd[i][j] = 0.0;
    }
  }

  /* global conversion */
  Q[1][1] = -q->q[_Q1_];
  Q[2][1] = -q->q[_Q2_];
  Q[3][1] = -q->q[_Q3_];

  Q[1][2] =  q->q[_Q0_];
  Q[2][2] =  q->q[_Q3_];
  Q[3][3] = -q->q[_Q2_];

  Q[1][3] = -q->q[_Q3_];
  Q[2][3] =  q->q[_Q0_];
  Q[3][3] =  q->q[_Q1_];

  Q[1][4] =  q->q[_Q2_];
  Q[2][4] = -q->q[_Q1_];
  Q[3][4] =  q->q[_Q0_];

  /* omega = 2*Q*quat_dot */
  for (i=1; i<=N_CART; ++i) {
    q->ad[i] = 0.0;
    for (j=1; j<=N_QUAT; ++j)
      q->ad[i] += 2.0*Q[i][j]*q->qd[j];
  }

  /* global conversion */
  Qd[1][1] = -q->qd[_Q1_];
  Qd[2][1] = -q->qd[_Q2_];
  Qd[3][1] = -q->qd[_Q3_];

  Qd[1][2] =  q->qd[_Q0_];
  Qd[2][2] =  q->qd[_Q3_];
  Qd[3][3] = -q->qd[_Q2_];

  Qd[1][3] = -q->qd[_Q3_];
  Qd[2][3] =  q->qd[_Q0_];
  Qd[3][3] =  q->qd[_Q1_];

  Qd[1][4] =  q->qd[_Q2_];
  Qd[2][4] = -q->qd[_Q1_];
  Qd[3][4] =  q->qd[_Q0_];

  /* omega_dot = 2*Q*quat_ddot + 2*Qd*quat_dot */
  for (i=1; i<=N_CART; ++i) {
    q->add[i] = 0.0;
    for (j=1; j<=N_QUAT; ++j)
      q->add[i] += (2.0*Qd[i][j]*q->qd[j] + 2.0*Q[i][j]*q->qdd[j]);
  }

}




static Matrix 
SL_inertiaMatrix(SL_Jstate *lstate, SL_Cstate *cbase, SL_quat *obase, SL_endeff *leff) 

{
  static int firsttime = TRUE;
  static Matrix rbdM;
  static Vector rbdCG;
  SL_uext ux[N_DOFS+1];

  if (firsttime) {
    firsttime = FALSE;
    
    rbdM  = my_matrix(1,N_DOFS+2*N_CART,1,N_DOFS+2*N_CART);
    rbdCG = my_vector(1,N_DOFS+2*N_CART);

  }
  
  SL_ForDynComp(lstate,cbase,obase,ux,leff,rbdM,rbdCG);

  return rbdM;

}

/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step
\date  April 2014
   
\remarks 

Given the time to go, the current state is updated to the next state
using min jerk splines

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          x,xd,xdd : the current state, vel, acceleration
 \param[in]          t,td,tdd : the target state, vel, acceleration
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[in]          x_next,xd_next,xdd_next : the next state after dt

 ******************************************************************************/
static int 
min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
		    double t_togo, double dt,
		    double *x_next, double *xd_next, double *xdd_next)

{
  double t1,t2,t3,t4,t5;
  double tau,tau1,tau2,tau3,tau4,tau5;
  int    i,j;

  // a safety check
  if (dt > t_togo || dt <= 0) {
    return FALSE;
  }

  t1 = dt;
  t2 = t1 * dt;
  t3 = t2 * dt;
  t4 = t3 * dt;
  t5 = t4 * dt;

  tau = tau1 = t_togo;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  // calculate the constants
  const double dist   = t - x;
  const double p1     = t;
  const double p0     = x;
  const double a1t2   = tdd;
  const double a0t2   = xdd;
  const double v1t1   = td;
  const double v0t1   = xd;
  
  const double c1 = 6.*dist/tau5 + (a1t2 - a0t2)/(2.*tau3) - 
    3.*(v0t1 + v1t1)/tau4;
  const double c2 = -15.*dist/tau4 + (3.*a0t2 - 2.*a1t2)/(2.*tau2) +
    (8.*v0t1 + 7.*v1t1)/tau3; 
  const double c3 = 10.*dist/tau3+ (a1t2 - 3.*a0t2)/(2.*tau) -
    (6.*v0t1 + 4.*v1t1)/tau2; 
  const double c4 = xdd/2.;
  const double c5 = xd;
  const double c6 = x;
  
  *x_next   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
  *xd_next  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
  *xdd_next = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;
  
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_state_machine
\date  March 2019
\remarks 

 parses the information about a state machine from a simple text file

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname : file name of file with state machine information,
                        by default, assumed in the PREFS director, ending 
                        in <filename>.sm

   format:

   name_of_state
   pose_x_X
   pose_x_Y
   pose_x_Z
   use_orient
   pose_q_Q0
   pose_q_Q1
   pose_q_Q2
   pose_q_Q3
   movement_duration
   gripper_width_start
   gripper_width_end
   gripper_force_start
   gripper_force_end
   cart_force_start
   cart_force_end
   cart_moment_start
   cart_moment_end
   use_default_gains
   cart_gain_x_X
   cart_gain_x_Y
   cart_gain_x_Z
   cart_gain_xd_X
   cart_gain_xd_Y
   cart_gain_xd_Z
   cart_gain_a_A
   cart_gain_a_B
   cart_gain_a_G
   cart_gain_ad_A
   cart_gain_ad_B
   cart_gain_ad_G
   controller_name

   some special characters:  


 ******************************************************************************/
static int
read_state_machine(char *fname) {

  int j,i,rc;
  char   string[200];
  FILE  *in;
  int    n_read;
  int    n_parms = 32; // number of parameters per state
  StateMachineTarget sm_temp;
  

  // open the file and strip all comments
  sprintf(string,"%s%s",PREFS,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }


  // read state-by-state until end-of-file
  n_states_sm = 0;
  do {
    n_read = fscanf(in,"%s  %lf %lf %lf  %d %lf %lf %lf %lf  %lf  %lf %lf %lf %lf  %lf %lf %lf %lf  %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s",
		    sm_temp.state_name,
		    &(sm_temp.pose_x[_X_]),
		    &(sm_temp.pose_x[_Y_]),
		    &(sm_temp.pose_x[_Z_]),
		    &(sm_temp.use_orient),
		    &(sm_temp.pose_q[_Q0_]),
		    &(sm_temp.pose_q[_Q1_]),
		    &(sm_temp.pose_q[_Q2_]),
		    &(sm_temp.pose_q[_Q3_]),
		    &(sm_temp.movement_duration),
		    &(sm_temp.gripper_width_start),
		    &(sm_temp.gripper_width_end),		    
		    &(sm_temp.gripper_force_start),
		    &(sm_temp.gripper_force_end),		    
		    &(sm_temp.cart_force_start),
		    &(sm_temp.cart_force_end),		    
		    &(sm_temp.cart_moment_start),
		    &(sm_temp.cart_moment_end),
		    &(sm_temp.use_default_gains),		    
		    &(sm_temp.cart_gain_x[_X_]),
		    &(sm_temp.cart_gain_x[_Y_]),
		    &(sm_temp.cart_gain_x[_Z_]),
		    &(sm_temp.cart_gain_xd[_X_]),
		    &(sm_temp.cart_gain_xd[_Y_]),
		    &(sm_temp.cart_gain_xd[_Z_]),
		    &(sm_temp.cart_gain_a[_A_]),
		    &(sm_temp.cart_gain_a[_B_]),
		    &(sm_temp.cart_gain_a[_G_]),
		    &(sm_temp.cart_gain_ad[_A_]),
		    &(sm_temp.cart_gain_ad[_B_]),
		    &(sm_temp.cart_gain_ad[_G_]),
		    sm_temp.controller_name
		    );
    if (n_read == n_parms) {
      if (n_states_sm < MAX_STATES_SM) {
	targets_sm[++n_states_sm] = sm_temp;
      } else {
	// should be unlikely to happen ever
	printf("Error: ran out of memory for state machine targets\n");
      }
    }
  } while (n_read == n_parms);

  printf("Read %d states for the state machine\n",n_states_sm);
  
  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  print_sm_state
\date  Feb 2019
   
\remarks 

       prints the current state such that it can be copied as sm target

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void 
print_sm_state(void)
{
  int i;

  printf("state_name ");

  for (i=1; i<=N_CART; ++i)
    printf("%5.3f ",cart_des_state[HAND].x[i]);

  printf("1 "); // use_orient

  for (i=1; i<=N_QUAT; ++i)
    printf("%5.3f ",cart_des_orient[HAND].q[i]);

  printf("2 "); // movement duration

  printf("%5.3f ",misc_sensor[G_WIDTH]);
  printf("%5.3f ",misc_sensor[G_WIDTH]);

  printf("0.0 0.0 "); // gripper force start and end
  printf("0.0 0.0 "); // cart force start and end
  printf("0.0 0.0 "); // cart moment start and end

  printf("1 "); // use_default_gains
  printf("0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 "); // gains

  printf("%s",controller_names[SIMPLE_IMPEDANCE_JT]);
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step_quat
\date  March 2019
   
\remarks 

 min jerk next step interpolation between two quaternions. Currently, this 
 assunes zero quaternion velocity/acceleration at the target quaternion.
 This algorithms is not entirely perfect, but should be reasonable in terms
 of spherical interpolation. It uses the SLERP algorithm suggested in 1985
 by Shoemake with the following algorithm:

 q_next = q_start*sin[(1-u)*theta]/sin[theta] + q_end*sin[u*theta]/sin[theta]

 q_start'*q_end = cos(theta)

 If we vary u from 0 to 1 as a min jerk trajectory, the entire orientation
 trajectory will be smooth as well (although not perfectly min jerk). In matlab,
 it was tested that even updating theta on every time step works and creates
 nice smooth unit norm quaternions

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          q_current: the current orientation as quaterion
 \param[in]          q_target : the target orientation as quaterion
 \param[in,out]      s        : vector of s,sd,sdd which does min jerk from 1 to zero
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[out]         q_next   : the next state quaternion with all derivatives

 ******************************************************************************/
static int
min_jerk_next_step_quat (SL_quat q_current, SL_quat q_target, double *s,
			 double t_togo, double dt, SL_quat *q_next)

{
  int    i,j;
  double theta; // angle between current and target quaternion
  double aux;
  double ridge = 1e-10;

  // make sure numerical issues of quaternion inner product cannot bother us
  aux = vec_mult_inner_size(q_current.q,q_target.q,N_QUAT);
  if (aux > 1)
    aux = 1;
  else if (aux < -1)
    aux = -1;
  theta = acos( aux );

  // current and target are identical
  if (theta == 0){
    for (i=1; i<=N_QUAT; ++i) {
      q_next->q[i] = q_current.q[i];
      q_next->qd[i] = 0.0;
      q_next->qdd[i] = 0.0;
    }
    return TRUE;
  }

  // min jerk for the indicator variable
  min_jerk_next_step(s[1],s[2],s[3],1.0,0.0,0.0,t_togo,dt,&(s[1]),&(s[2]),&(s[3]));

  // interpolate quaternions based on indicator variable

  // numerical robustness
  if (fabs(sin(theta)) < ridge)
    aux = ridge;
  else
    aux = sin(theta);
  
  for (i=1; i<=N_QUAT; ++i) {
    q_next->q[i] = sin(theta*(1.0-s[1]))/aux*q_current.q[i] + sin(theta*s[1])/aux*q_target.q[i];
  }

  // fill in derivatives from numerical differentiation
  for (j=1; j<=N_QUAT; ++j){
    q_next->qd[j]  =  (q_next->q[j] - q_current.q[j]) / dt;
    q_next->qdd[j] =  (q_next->qd[j] - q_current.qd[j]) / dt;
  }

  //  fprintf(fp,"%f %f %f   %f %f %f %f\n",s[1],s[2],s[3],q_next->q[1],q_next->q[2],q_next->q[3],q_next->q[4]);
  
  return TRUE;

}


