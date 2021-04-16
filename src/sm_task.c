/*============================================================================
==============================================================================
                      
                              sm_task.c
 
==============================================================================
Remarks:

      a general state machine task with scripted state machine

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
#include "utility_macros.h"
#include "data_collection_lib.h"
#include "time.h"

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
   MODEL_IMPEDANCE_JT,
   N_CONT
  };

static int n_controllers = N_CONT-1;

char controller_names[][100]=
  {
   {"dummy"},
   {"SimpleImpedanceJt"},
   {"ModelImpedanceJt"},
  };

enum FTEceptionAction
  {
   CONT=1,
   ABORT,
   LAST
  };

enum RelativeOption
  {
   ABS = 0,
   REL,
   RELREF
  };

enum CoordinateFrame
  {
   WORLD_FRAME = 0,
   REFERENCE_FRAME
  };

enum ExitOption
  {
   NO_EXIT = 0,
   POS_EXIT,
   ORIENT_EXIT,
   POS_ORIENT_EXIT,
   FORCE_EXIT,
   MOMENT_EXIT,
   FORCE_MOMENT_EXIT,
   EXIT_AFTER_TABLE,
   EXIT_AFTER_FUNCTION_CALL_SUCCESS,
   GRASP_PERTURBATION
  };

enum ManipulationFrame
  {
    ABS_FRAME=0,
    REF_FRAME,
    BASE_REF_FRAME    
  };
		 
enum FunctionCalls
  {
    NO_FUNC = 0,
    ZERO_FT,
    NEXT_TABLE_DELTA,
    GRIPPER,
    SEND_POSE_EST_DATA,
    RECEIVE_POSE_CORRECTION,
    RESET_TO_BASE_POSE,
    ADD_CURRENT_TO_STATISTICS,
  };
		 

#define MAX_FUNCTION_ARGS 10
typedef struct StateMachineTarget {
  char   state_name[100];
  char   next_state_name[100];
  int    next_state_id;
  int    manipulation_frame;
  int    function_call;
  int    n_function_args;
  double function_args[MAX_FUNCTION_ARGS+1];
  double movement_duration;
  int    pose_x_is_relative;
  double pose_x[N_CART+1];
  int    use_orient;
  int    pose_q_is_relative;
  double pose_q[N_QUAT+1];
  int    gripper_start_active;
  int    gripper_end_active;  
  int    gripper_width_start_is_relative;
  double gripper_width_start;
  int    gripper_width_end_is_relative;  
  double gripper_width_end;
  double gripper_force_start;
  double gripper_force_end;
  double force_des_gain;
  double force_des[N_CART+1];
  double moment_des_gain;
  double moment_des[N_CART+1];
  int    ft_exception_action;
  double max_wrench[2*N_CART+1];
  double cart_gain_x_scale[N_CART+1]; // diagonal of matrix in ref frame
  double cart_gain_xd_scale[N_CART+1]; // diagonal of matrix in ref frame
  double cart_gain_a_scale[N_CART+1];  // diagonal of matrix in ref frame
  double cart_gain_ad_scale[N_CART+1]; // diagonal of matrix in ref frame
  Matrix cart_gain_x_scale_matrix;  //  full matrix in world frame
  Matrix cart_gain_xd_scale_matrix; // full matrix in world frame
  Matrix cart_gain_a_scale_matrix;  // full matrix in world frame
  Matrix cart_gain_ad_scale_matrix; // full matrix in world frame
  double cart_gain_integral;
  char   controller_name[100];
  int    exit_condition;
  double exit_timeout;
  double err_pos; // abs error in position x,y,z
  double err_orient; // abs error in rad
  double err_force; // abs error in force
  double err_moment; // abs error in moment
} StateMachineTarget;

#define MAX_STATES_SM 10000
static StateMachineTarget targets_sm[MAX_STATES_SM+1];
static StateMachineTarget current_target_sm;

// these stats need to be accessible to data collection
// TODO: communicate them rather than shareing in global variables
double reference_state_pose_x[N_CART+1];
double reference_state_pose_q[N_QUAT+1];
double reference_state_pose_x_base[N_CART+1];
double reference_state_pose_q_base[N_QUAT+1];
double reference_state_pose_delta_x_table[MAX_STATES_SM+1][N_CART+1];
double reference_state_pose_delta_q_table[MAX_STATES_SM+1][N_QUAT+1];
int    current_state_pose_delta = 0;
extern int        global_sample_id;
double grasp_perturbation_x[N_CART+1];
double grasp_perturbation_q[N_QUAT+1];
static int apply_grasp_perturbation = FALSE;    

// a big matrix to collect some statistics of visual learning experiments
// the number of rows used will be equivalent to n_states_pose_delta at most
Matrix statistics_matrix;

static int    allow_base_reference_frame=FALSE;
static int    n_states_sm = 0;
static int    n_states_pose_delta = 0;
static int    current_state_sm = 0;
static double speed_mult = 1.0;
static int    current_controller = SIMPLE_IMPEDANCE_JT;
static int    default_controller = SIMPLE_IMPEDANCE_JT;
static int    function_call_success;

/* Cartesian orientation representation with a rotation around an axis */
typedef struct { 
  double phi;          /* rotation angle */
  double r[N_CART+1];  /* rotation axis  */
} My_Crot;

/* local variables */
static double     time_step;
static double     cref[N_ENDEFFS*6+1];
SL_Cstate  ctarget[N_ENDEFFS+1];
static SL_Cstate  cdes[N_ENDEFFS+1];
SL_quat    ctarget_orient[N_ENDEFFS+1];
static My_Crot    ctarget_rot[N_ENDEFFS+1];  
static SL_quat    cdes_orient[N_ENDEFFS+1];
static SL_quat    cdes_start_orient[N_ENDEFFS+1];
static double     des_gripper_width;
static double     corient_error[N_ENDEFFS*3+1];
static int        stats[N_ENDEFFS*6+1];
static SL_DJstate target[N_DOFS+1];
static SL_DJstate last_target[N_DOFS+1];
static int        firsttime = TRUE;
static double     start_time     = 0;
static double     default_gain   = 700;  // was 450
static double     default_gain_orient = 40;  // was 40
static double     default_gain_integral = 0.25;
static double     gain_integral = 0.0;
static double     gain_force = 0.1;
static int        found_recurrance = FALSE;
static int        run_table = FALSE;

static SL_Cstate  ball_state;

static double u_delta_switch[N_DOFS+1];

static int    state_machine_state = INIT_SM_TARGET;

static double s[3+1]; // indicator for min jerk in orientation space

static double     default_cart_gain_x_scale[2*N_CART+1];
static double     default_cart_gain_a_scale[2*N_CART+1];

static double pos_error;
static double orient_error;
static double pos_tracking_error;
static double orient_tracking_error;

double pos_error_vector[N_CART+1];
double orient_error_quat[N_QUAT+1];

double pos_tracking_error_vector[N_CART+1];
double orient_tracking_error_quat[N_QUAT+1];

double max_pos_tracking_error;
double min_pos_tracking_error;
double mean_pos_tracking_error;

double max_orient_tracking_error;
double min_orient_tracking_error;
double mean_orient_tracking_error;

// for no_user_interaction_flag
char   sm_file_name[200];
int    sm_run_table = FALSE;
extern int no_user_interaction_flag;

/* global functions */
void add_sm_task(void);
extern void init_sm_controllers(void);
extern int  cartesianImpedanceSimpleJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double **gain_x_scale,
			   double **gain_xd_scale,
			   double **gain_a_scale,
			   double **gain_ad_scale);
extern int  cartesianImpedanceModelJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double **gain_x_scale,
			   double **gain_xd_scale,
			   double **gain_a_scale,
			   double **gain_ad_scale);


/* local functions */
static int    init_sm_task(void);
static int    run_sm_task(void);
static int    change_sm_task(void);
static int    teach_target_pose(void);
static int    min_jerk_next_step (double x,double xd, double xdd, double t, double td, double tdd,
				  double t_togo, double dt,
				  double *x_next, double *xd_next, double *xdd_next);
static int    min_jerk_next_step_quat (SL_quat q_current, SL_quat q_target, double *s,
				       double t_togo, double dt, SL_quat *q_next);
static int    min_jerk_next_step_quat_new (SL_quat q, SL_quat q_target,
					   double t_togo, double dt, SL_quat *q_next);
static int    read_state_machine(char *fname);
static void   print_sm_state(void);
static void   save_statistics_matrix(void);
static void   print_sm_stats(void);
static void   assignCurrentSMTarget(StateMachineTarget smt,
				    double *ref_x,
				    double *ref_q,
				    SL_Cstate *ct,
				    SL_quat   *cto,
				    StateMachineTarget *smc);
static int    functionCall(int id, int initial_call, int *success) ;



/*****************************************************************************
******************************************************************************
Function Name	: add_sm_task
Date		: Feb 2019
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_sm_task( void )
{
  int i, j;
  char string[100];
  
  addTask("State Machine Task", init_sm_task, 
	  run_sm_task, change_sm_task);

  addToMan("print_sm_state","prints the current state suitable for state machine",print_sm_state);
  addToMan("save_statistics","saves statistics matrix to file",save_statistics_matrix);
  addToMan("sm_stats","prints some stats of state machine",print_sm_stats);     

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

    sprintf(string,"%s_cdes_q0",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].q[_Q0_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q1",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].q[_Q1_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q2",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].q[_Q2_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q3",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].q[_Q3_]),string,"-", DOUBLE,FALSE);

    sprintf(string,"%s_cdes_q0d",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qd[_Q0_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q1d",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qd[_Q1_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q2d",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qd[_Q2_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q3d",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qd[_Q3_]),string,"-", DOUBLE,FALSE);

    sprintf(string,"%s_cdes_q0dd",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qdd[_Q0_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q1dd",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qdd[_Q1_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q2dd",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qdd[_Q2_]),string,"-", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_q3dd",cart_names[i]);
    addVarToCollect((char *)&(cdes_orient[i].qdd[_Q3_]),string,"-", DOUBLE,FALSE);

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
      
    sprintf(string,"%s_cdes_xd",cart_names[i]);
    addVarToCollect((char *)&(cdes[i].xd[_X_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_yd",cart_names[i]);
    addVarToCollect((char *)&(cdes[i].xd[_Y_]),string,"m", DOUBLE,FALSE);
    sprintf(string,"%s_cdes_zd",cart_names[i]);
    addVarToCollect((char *)&(cdes[i].xd[_Z_]),string,"m", DOUBLE,FALSE);
      
      
      
  }
    
  sprintf(string,"sm_pos_error");
  addVarToCollect((char *)&(pos_error),string,"m", DOUBLE,FALSE);
  sprintf(string,"sm_orient_error");
  addVarToCollect((char *)&(orient_error),string,"rad", DOUBLE,FALSE);
  sprintf(string,"sm_state_id");
  addVarToCollect((char *)&(current_state_sm),string,"-", INT,FALSE);
    
  updateDataCollectScript();
    
  
}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_sm_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_sm_task(void)
{
  int    j, i;
  char   string[100];
  //  static char   fname[100] = "qsfp_vl_collection.sm";
  static char   fname[100] = "waterproof_vl_collection.sm";  
  int    ans;
  int    flag = FALSE;
  static int firsttime = TRUE;
  static int pert = 0;

  
  if (firsttime) {

    // zero out variables
    bzero((char *)&cref,sizeof(cref));
    bzero((char *)&ctarget,sizeof(ctarget));
    bzero((char *)&ctarget_orient,sizeof(ctarget_orient));

    for (i=1; i<=MAX_STATES_SM; ++i) {
      targets_sm[i].cart_gain_x_scale_matrix = my_matrix(1,N_CART,1,N_CART);
      targets_sm[i].cart_gain_xd_scale_matrix = my_matrix(1,N_CART,1,N_CART);
      targets_sm[i].cart_gain_a_scale_matrix = my_matrix(1,N_CART,1,N_CART);
      targets_sm[i].cart_gain_ad_scale_matrix = my_matrix(1,N_CART,1,N_CART);
    }

    // allocate just ample of columns to avoid thinking about how many
    statistics_matrix = my_matrix(1, MAX_STATES_SM, 1, 100);

    firsttime = FALSE;
    
  }

  // the sm_controllers
  init_sm_controllers();

  // zero the delta command from controller switches
  for (i=1; i<=N_DOFS; ++i)
    u_delta_switch[i] = 0.0;

  
  // check whether any other task is running 
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Task can only be started if no other task is running!\n");
    return FALSE;
  }

  // speed change?
  if (!no_user_interaction_flag ) { // no interactive input
    get_double("Speed multiplier?",speed_mult,&speed_mult);
    if (speed_mult <= 0 || speed_mult > 2)
      speed_mult = 1.0;
  }
  
  // read state machine
  if (!no_user_interaction_flag ) { // no interactive input
    ans = TRUE;
    get_int("Read state machine from file?",ans,&ans);
  } else {
    strcpy(fname,sm_file_name);
    ans = TRUE;
  }
  if (ans) {
    if (no_user_interaction_flag ) { // no interactive input
      if (!read_state_machine(fname))
	return FALSE;
    } else {
      if (get_string("File name of state machine in prefs/",fname,fname)) {
	int sl = strlen(fname);
	if (strcmp(&(fname[sl-3]),".sm") != 0)
	  strcat(fname,".sm");
	if (!read_state_machine(fname))
	  return FALSE;
      }
    }
  }
  if (n_states_sm <= 0) {
    printf("No valid state machine found\n");
    return FALSE;
  }
  if (found_recurrance && n_states_pose_delta > 0) {
    if (no_user_interaction_flag) {
      run_table = sm_run_table;
    } else {
      get_int("Run table of reference perturbations?",run_table,&run_table);
    }
  } else {
    run_table = FALSE;
  }
  
  // check whether systematic variations should be run

  // perturb reference?
  if (!run_table && !no_user_interaction_flag) {
    get_int("Perturb reference?",pert,&pert);
    if (pert == 1) {
      double dx[N_CART+1];
      double dq[N_QUAT+1];
      double std = sqrt(1./3.);
      double aux = 0;
      double angle;
      
      dx[_X_] = uniform(0.0,std*0.001);
      dx[_Y_] = uniform(0.0,std*0.001);
      dx[_Z_] = uniform(0.0,std*0.002);
      dq[_Q1_] = uniform(0.0,std*1.0);
      dq[_Q2_] = uniform(0.0,std*1.0);
      dq[_Q3_] = uniform(0.0,std*1.0);
      
      angle = 5./180*PI;
      dq[_Q0_] = cos(angle/2.);
      
      for (i=_Q1_; i<=_Q3_; ++i)
	aux += sqr(dq[i]);
      aux = sqrt(aux);
      for (i=_Q1_; i<=_Q3_; ++i) {
	dq[i] /= aux;
	dq[i] *= sin(angle/2.);
      }
      
      printf("delta x: %f %f %f\n",dx[1],dx[2],dx[3]);
      printf("delta q: %f %f %f %f\n",dq[1],dq[2],dq[3],dq[4]);
      
      for (i=1; i<=N_CART; ++i)
	reference_state_pose_x[i] = reference_state_pose_x_base[i] + dx[i];
      
      quatMult(reference_state_pose_q_base,dq,reference_state_pose_q);
      
    }
  }

  // go to a save posture 
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  bzero((char *)&(last_target[1]),N_DOFS*sizeof(last_target[1]));
  for (i=1; i<=N_DOFS; ++i) {
    target[i] = joint_default_state[i];
    last_target[i] = joint_default_state[i];
  }

  /*
  target[J1].th  = -0.023;
  target[J2].th  =  0.29;
  target[J3].th  =  0.029;
  target[J4].th  = -1.964;
  target[J5].th  = 0.092;
  target[J6].th  = 2.237;
  target[J7].th  = -0.867;
  */

  des_gripper_width = 0.05;
  sendGripperMoveCommand(des_gripper_width,0.1);

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
    printf("cdes_orient %f\n",cdes_orient[HAND].q[j]);
  }

  // ready to go
  if (!no_user_interaction_flag) {
    ans = 999;
    while (ans == 999) {
      if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
	return FALSE;
    }

    if (ans != 1) 
      return FALSE;
  }  

  // reclibrate the gripper F/T offsets
  sendCalibrateFTCommand();
  taskDelay(100);

  time_step = 1./(double)task_servo_rate;
  start_time = task_servo_time;
  state_machine_state = INIT_SM_TARGET;
  current_state_pose_delta = 0;
  current_state_sm = 0;
  current_target_sm = targets_sm[current_state_sm];
  no_user_interaction_flag = FALSE;
  mat_zero(statistics_matrix);
  
  scd();
  
  
  return TRUE;
  
  }
  
/*****************************************************************************
******************************************************************************
  Function Name	: run_sm_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_sm_task(void)
{
  int j, i;
  double sum=0;
  double dist;
  double aux;
  static double time_to_go;
  static int n_mean = 0;
  float pos[N_CART+1+1]; // one extra element for radius
  double gripper_move_threshold = 1e-8;
  static int wait_ticks=0;
  int    no_gripper_motion = TRUE;
  char   msg[1000];
  int    update_u_delta_switch = FALSE;
  int    ft_exception_flag = FALSE;
  char   string[200];
  float  b[N_CART*3+1];
  
  switch (state_machine_state) {

  case INIT_SM_TARGET:
    
    // check whether to end state machine

    if (current_target_sm.exit_condition == EXIT_AFTER_TABLE && run_table && current_state_pose_delta == n_states_pose_delta) {
      sprintf(msg,"All done with table!\n");
      logMsg(msg,0,0,0,0,0,0);
      freeze();
      return TRUE;
    }

    // a special adjustment for an explicit jump to a a next_state out of sequence
    if (current_state_sm != 0 && current_target_sm.next_state_id != 0) {

      if ((run_table && current_state_pose_delta < n_states_pose_delta) || !run_table) {
	// this simple adjustment allows continuing with the normal sequence below
	current_state_sm = current_target_sm.next_state_id-1;
	scd();
      }

    }


    if (current_state_sm < n_states_sm) {
      
      ++current_state_sm;
      if (current_state_sm == 1) { // just for clearer print-outs
	logMsg("\n",0,0,0,0,0,0);
      }

      // are we running through the table of reference perturbations?: this requires functionCall to
      // come before asssignCurrentSMTarget.
      if ((run_table && current_state_pose_delta < n_states_pose_delta &&
	   targets_sm[current_state_sm].function_call == NEXT_TABLE_DELTA) ||
	  targets_sm[current_state_sm].function_call == RESET_TO_BASE_POSE ||
	  targets_sm[current_state_sm].function_call == GRASP_PERTURBATION
	  ) {
	
	functionCall(targets_sm[current_state_sm].function_call,TRUE,&function_call_success);
	
      }

      // assign to simpler variable and take into account the reference frame of the state
      if (targets_sm[current_state_sm].manipulation_frame == BASE_REF_FRAME && allow_base_reference_frame) {
	assignCurrentSMTarget(targets_sm[current_state_sm],
			      reference_state_pose_x_base,
			      reference_state_pose_q_base,
			      ctarget,
			      ctarget_orient,
			      &current_target_sm);
      } else {
	assignCurrentSMTarget(targets_sm[current_state_sm],
			      reference_state_pose_x,
			      reference_state_pose_q,
			      ctarget,
			      ctarget_orient,
			      &current_target_sm);
      }

      if (current_target_sm.cart_gain_integral != 0) 
	sprintf(msg,"    %d.%-30s with %sInt\n",
		current_state_sm,current_target_sm.state_name,current_target_sm.controller_name);
      else
	sprintf(msg,"    %d.%-30s with %s\n",
		current_state_sm,current_target_sm.state_name,current_target_sm.controller_name);
      logMsg(msg,0,0,0,0,0,0);
      
    } else {
      
      sprintf(msg,"All done!\n");
      logMsg(msg,0,0,0,0,0,0);
      freeze();
      return TRUE;
      
    }

    // check whether there is a function call, with special exception for NEXT_TABLE_DELTA
    if (current_target_sm.function_call != NEXT_TABLE_DELTA && current_target_sm.function_call != RESET_TO_BASE_POSE)
      functionCall(current_target_sm.function_call,TRUE,&function_call_success);

    // assign relevant variables from state machine state array
    time_to_go = current_target_sm.movement_duration/speed_mult;

    // need to memorize the cart orient start for min jerk interpolation
    cdes_start_orient[HAND] = cdes_orient[HAND];

    // gripper movement: only if gripper states have changed
    no_gripper_motion = FALSE;
    if (current_state_sm > 1 && current_target_sm.gripper_start_active) {
      if (current_target_sm.gripper_width_start == targets_sm[current_state_sm-1].gripper_width_end &&
	  current_target_sm.gripper_force_start == targets_sm[current_state_sm-1].gripper_force_end) {
	no_gripper_motion=TRUE; 
      }
    } else if (!current_target_sm.gripper_start_active) {
	no_gripper_motion=TRUE;       
    }

    if (!no_gripper_motion) {
      // give move command to gripper to desired position if width is larger than current width
      if (current_target_sm.gripper_width_start > misc_sensor[G_WIDTH] ||
	  current_target_sm.gripper_force_start == 0) {
	
	sendGripperMoveCommand(current_target_sm.gripper_width_start,0.1);
	
      } else { // or close gripper with force control otherwise
	
	sendGripperGraspCommand(current_target_sm.gripper_width_start,
				0.1,
				current_target_sm.gripper_force_start,
				0.08,
				0.08);
	
      }
      wait_ticks = 100; // need to give non-real-time gripper thread a moment to get started
      state_machine_state = GRIPPER_START;
    } else {
      state_machine_state = MOVE_TO_TARGET;
      n_mean = 0;
    }
    
    // prepare min jerk for orientation space: s is an interpolation variable 
    s[1] = s[2] = s[3] = 0;

    // which controller?
    for (i=1; i<=n_controllers; ++i) {
      if (strcmp(current_target_sm.controller_name,controller_names[i]) == 0) {
	current_controller = i;
	update_u_delta_switch = TRUE;
	break;
      }
    }
    if (i > n_controllers ) { // did not find valid controller
      printf("State %s has no valid controller --- abort\n",current_target_sm.state_name);
      freeze();
      return FALSE;
    }

    break;
    
    
  case GRIPPER_START:
    
    if (--wait_ticks < 0) {
      if (misc_sensor[G_MOTION] == 0) {
	state_machine_state = MOVE_TO_TARGET;
	n_mean = 0;	
      }
    }
    
    break;
    
    
  case MOVE_TO_TARGET:

    // check for function call success
    functionCall(current_target_sm.function_call,FALSE,&function_call_success);
    if (current_target_sm.exit_condition == EXIT_AFTER_FUNCTION_CALL_SUCCESS && function_call_success) {

      for (i=1; i<=N_CART; ++i) {
	cdes[HAND].xd[i]  = 0.0;
	cdes[HAND].xdd[i] = 0.0;
	cdes_orient[HAND].ad[i]  = 0.0;
	cdes_orient[HAND].add[i]  = 0.0;	
      }

      for (i=1; i<=N_QUAT; ++i) {
	cdes_orient[HAND].qd[i]  = 0.0;
	cdes_orient[HAND].qdd[i]  = 0.0;	
      }

      time_to_go = 0;

    }

    // check for exceeding max force/torque
    if (fabs(misc_sensor[C_FX]) > current_target_sm.max_wrench[_X_]) {
      sprintf(string,"max force X exceeded (|%6.3f| > %6.3f)\n",misc_sensor[C_FX],current_target_sm.max_wrench[_X_]);
      logMsg(string,0,0,0,0,0,0);
      ft_exception_flag = TRUE;
    }

    if (fabs(misc_sensor[C_FY]) > current_target_sm.max_wrench[_Y_]) {
      sprintf(string,"max force Y exceeded (|%6.3f| > %6.3f)\n",misc_sensor[C_FY],current_target_sm.max_wrench[_Y_]);
      logMsg(string,0,0,0,0,0,0);
      ft_exception_flag = TRUE;
    }

    if (fabs(misc_sensor[C_FZ]) > current_target_sm.max_wrench[_Z_]) {
      sprintf(string,"max force Z exceeded (|%6.3f| > %6.3f)\n",misc_sensor[C_FZ],current_target_sm.max_wrench[_Z_]);
      logMsg(string,0,0,0,0,0,0);
      ft_exception_flag = TRUE;
    }

    if (fabs(misc_sensor[C_MX]) > current_target_sm.max_wrench[N_CART+_A_]) {
      sprintf(string,"max torque A exceeded (|%6.3f| > %6.3f)\n",misc_sensor[C_MX],current_target_sm.max_wrench[N_CART+_A_]);
      logMsg(string,0,0,0,0,0,0);
      ft_exception_flag = TRUE;
    }

    if (fabs(misc_sensor[C_MY]) > current_target_sm.max_wrench[N_CART+_B_]) {
      sprintf(string,"max torque B exceeded (|%6.3f| > %6.3f)\n",misc_sensor[C_MY],current_target_sm.max_wrench[N_CART+_B_]);
      logMsg(string,0,0,0,0,0,0);
      ft_exception_flag = TRUE;
    }

    if (fabs(misc_sensor[C_MZ]) > current_target_sm.max_wrench[N_CART+_G_]) {
      sprintf(string,"max torque G exceeded (|%6.3f| > %6.3f)\n",misc_sensor[C_MZ],current_target_sm.max_wrench[N_CART+_G_]);
      logMsg(string,0,0,0,0,0,0);
      ft_exception_flag = TRUE;
    }

    if (ft_exception_flag) {

      for (i=1; i<=N_CART; ++i) {
	cdes[HAND].xd[i]  = 0.0;
	cdes[HAND].xdd[i] = 0.0;
	cdes_orient[HAND].ad[i]  = 0.0;
	cdes_orient[HAND].add[i]  = 0.0;	
      }

      time_to_go = 0;

      switch (current_target_sm.ft_exception_action) {

      case CONT: // not action needed for continue action
	break;

      case LAST: // switch to last sm state
	current_state_sm = n_states_sm - 1;
	break;

      default:
	freeze();
	return FALSE;
	  
      }

    } else {

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
      
      if (current_target_sm.use_orient) {
	SL_quat ttt;

	/*
	min_jerk_next_step_quat(cdes_start_orient[HAND], ctarget_orient[HAND], s,
				time_to_go, time_step, &(cdes_orient[HAND]));
	*/

	min_jerk_next_step_quat_new(cdes_orient[HAND],ctarget_orient[HAND],
				    time_to_go, time_step,&(cdes_orient[HAND]));

	/*
	print_vec_size("old",cdes_orient[HAND].q,4);
	print_vec_size("new",ttt.q,4);	
	print_vec_size("old -d",cdes_orient[HAND].qd,4);
	print_vec_size("new -d",ttt.qd,4);	
	print_vec_size("old -dd",cdes_orient[HAND].qdd,4);
	print_vec_size("new -dd",ttt.qdd,4);
	if (my_isnan(ttt.q[1]) == 1)
	  getchar();
	*/

      }
    }
    
    time_to_go -= time_step;

    // logging of position and orientation error

    // position error relative to target
    pos_error = 0.0;
    for (i=1; i<=N_CART; ++i) {
      pos_error_vector[i] = cart_state[HAND].x[i] - ctarget[HAND].x[i];
      pos_error += sqr(pos_error_vector[i]);
    }
    pos_error = sqrt(pos_error);
    
    // position tracking
    pos_tracking_error = 0.0;
    for (i=1; i<=N_CART; ++i) {
      pos_tracking_error_vector[i] = cdes[HAND].x[i] - cart_state[HAND].x[i];
      pos_tracking_error += sqr(pos_tracking_error_vector[i]);
    }
    pos_tracking_error = sqrt(pos_tracking_error);
    
    if (pos_tracking_error > max_pos_tracking_error)
      max_pos_tracking_error = pos_tracking_error;

    if (pos_tracking_error < min_pos_tracking_error)
      min_pos_tracking_error = pos_tracking_error;

    mean_pos_tracking_error = (mean_pos_tracking_error*(double)n_mean + pos_tracking_error)/(double)(n_mean+1);
    

    // orientation error relative to target
    quatRelative(ctarget_orient[HAND].q, cart_orient[HAND].q, orient_error_quat);
    orient_error = 2.0*fabs(acos(orient_error_quat[_Q0_]));
    if (orient_error > PI)
      orient_error = 2.*PI-orient_error;

    // orientation tracking error 
    quatRelative(cart_orient[HAND].q, cdes_orient[HAND].q, orient_tracking_error_quat);
    orient_tracking_error = 2.0*fabs(acos(orient_tracking_error_quat[_Q0_]));
    if (orient_tracking_error > PI)
      orient_tracking_error = 2.*PI-orient_tracking_error;

    if (orient_tracking_error > max_orient_tracking_error)
      max_orient_tracking_error = orient_tracking_error;
    
    if (orient_tracking_error < min_orient_tracking_error)
      min_orient_tracking_error = orient_tracking_error;

    mean_orient_tracking_error = (mean_orient_tracking_error*(double)n_mean + orient_tracking_error)/(double)(n_mean+1);

    n_mean += 1;
    

    // check whether there is an exit condtion which overules progressing 
    // to the next state
    
    if (time_to_go < 0 && current_target_sm.exit_condition && !ft_exception_flag) {
      int    exit_flag = TRUE;
      
      switch(current_target_sm.exit_condition)
	{
	case POS_EXIT:
	  if (pos_error > current_target_sm.err_pos)
	    exit_flag = FALSE;
	  break;
	  
	case ORIENT_EXIT:
	  if (orient_error > current_target_sm.err_orient)
	    exit_flag = FALSE;
	  break;
	  
	case POS_ORIENT_EXIT:
	  if (pos_error > current_target_sm.err_pos ||
	      orient_error > current_target_sm.err_orient)
	    exit_flag = FALSE;
	  break;

	case EXIT_AFTER_FUNCTION_CALL_SUCCESS:
	  if (!function_call_success)
	    exit_flag = FALSE;
	  break;

	}

      if (!exit_flag && fabs(time_to_go) < current_target_sm.exit_timeout)
	break;
      else if (fabs(time_to_go) >= current_target_sm.exit_timeout && current_target_sm.exit_condition != EXIT_AFTER_TABLE) {
	sprintf(msg,"Time out at  %f e_pos=%f e_orient=%f\n",time_to_go,pos_error,orient_error);
	logMsg(msg,0,0,0,0,0,0);
      }

      
    } // end check exit conditions
    

    // at this point we know that we will exit this state of the state machine    
    if (time_to_go < 0) {
      time_to_go = 0;

      no_gripper_motion = FALSE;
      if (current_target_sm.gripper_end_active) {
	if (current_target_sm.gripper_width_end ==
	    current_target_sm.gripper_width_start &&
	    current_target_sm.gripper_force_end ==
	    current_target_sm.gripper_force_start) {
	  no_gripper_motion=TRUE; 
	}
      } else {
	  no_gripper_motion=TRUE;
      }

      if (!no_gripper_motion) {
	if (current_target_sm.gripper_width_end > misc_sensor[G_WIDTH] ||
	    current_target_sm.gripper_force_end == 0) {
	  sendGripperMoveCommand(current_target_sm.gripper_width_end,0.1);
	} else {
	  sendGripperGraspCommand(current_target_sm.gripper_width_end,
				  0.1,
				  current_target_sm.gripper_force_end,
				  0.08,
				  0.08);
	}
	wait_ticks = 100; // need to give non-real-time gripper thread a moment to get started
	state_machine_state = GRIPPER_END;
      } else {
	//print_sm_stats();
	state_machine_state = INIT_SM_TARGET;
      }
    } // end time_to_go < 0

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


  // switch controllers

  // smooth controller switsches
  for (i=1; i<=N_DOFS; ++i) {

    // smooth out controller switches: Step 1: remember last uff
    if (update_u_delta_switch) {
      u_delta_switch[i] = joint_des_state[i].uff;
    }
  }

  switch (current_controller) {

  case SIMPLE_IMPEDANCE_JT:

    cartesianImpedanceSimpleJt(cdes, cdes_orient, joint_des_state, joint_opt_state, stats,
			       current_target_sm.cart_gain_integral,
			       current_target_sm.cart_gain_x_scale_matrix,
			       current_target_sm.cart_gain_xd_scale_matrix,
			       current_target_sm.cart_gain_a_scale_matrix,
			       current_target_sm.cart_gain_ad_scale_matrix);

    break;
    
  case MODEL_IMPEDANCE_JT:

    cartesianImpedanceModelJt(cdes, cdes_orient, joint_des_state, joint_opt_state, stats,
			      current_target_sm.cart_gain_integral,
			      current_target_sm.cart_gain_x_scale_matrix,
			      current_target_sm.cart_gain_xd_scale_matrix,
			      current_target_sm.cart_gain_a_scale_matrix,
			      current_target_sm.cart_gain_ad_scale_matrix);
    
    break;
    
  }

  // add force control if the controllers has force gains 
  double force_world[N_CART+1], torque_world[N_CART+1];
  double force_local[N_CART+1], torque_local[N_CART+1];  
  double offset[N_CART+1];
  double torque_correction[N_CART+1];

  for (j=1; j<=N_CART; ++j) {
    force_local[j]  = misc_sensor[C_FX-1+j];
    torque_local[j] = misc_sensor[C_MX-1+j];
  }

  // convert the F/T sensor into the compliance frame, which could be at the top of the
  // endeffector tool (e.g., connector) but also further down. Currently, only Z direction
  // is taking into account for shifting the compliance frame

  vec_zero_size(offset,N_CART);
  offset[_Z_] = endeff[HAND].x[_Z_]-(FL+FT_OFF_Z);
  vec_mult_outer_size(offset, force_local, N_CART, torque_correction);
  
  for (i=1; i<=N_CART; ++i) {
    torque_local[i] -= torque_correction[i];
  }

  // need F/T signals in world coordinates
  for (i=1; i<=N_CART; ++i) {
    force_world[i] = torque_world[i] = 0;
    for (j=1; j<=N_CART; ++j) {
      force_world[i]  += Alink[FLANGE][i][j] * force_local[j];
      torque_world[i] += Alink[FLANGE][i][j] * torque_local[j];
    }
  }

  // send to visualization
  float data[3*N_CART+1];
  for (i=1; i<=N_CART; ++i) {
    data[i]           = force_world[i];
    data[i+N_CART]    = torque_world[i];
    data[i+2*N_CART]  = cart_state[HAND].x[i];
  }
  sendUserGraphics("displayFTVector",&(data[1]),3*N_CART*sizeof(float));

  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=N_CART; ++j) {
      if (stats[j]) 
	joint_des_state[i].uff -= J[j][i]*
	  (current_target_sm.force_des[j]-force_world[j])*
	  current_target_sm.force_des_gain;
    }
  }

  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=N_CART; ++j) {
      if (stats[N_CART+j])
	joint_des_state[i].uff -= J[j+N_CART][i]*
	  (current_target_sm.moment_des[j]-torque_world[j])*
      	  current_target_sm.moment_des_gain;
    }
  }


  for (i=1; i<=N_DOFS; ++i) {

    // smooth out controller switches: Step 2: check the difference in uff
    if (update_u_delta_switch) {
      u_delta_switch[i] -= joint_des_state[i].uff;
    }

    joint_des_state[i].uff   += u_delta_switch[i]; // transient to avoid jumps from controller switch
    u_delta_switch[i] *= 0.995;
  }


  // visualize the cartesian desired as a ball
  for (i=1; i<=N_CART; ++i)
    pos[i] = cdes[HAND].x[i];
  pos[_Z_+1] = 0.005;
  sendUserGraphics("ballSize",&(pos[_X_]), (N_CART+1)*sizeof(float));
  /*
  b[1] = .5;
  b[2] =  0.0;
  b[3] = .4;
  b[4] = 0;
  b[5] = -1;
  b[6] = 0;
  b[7] = .1;
  b[8] = 0.05;
  b[9] = 0.01;


  sendUserGraphics("RectCuboid",&(b[1]), (N_CART*3)*sizeof(float));  
  */
  return TRUE;
  
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_sm_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_sm_task(void)
{
  int    j,i;
  char   string[100];
  double aux;


  get_double("Force gain",gain_force,&aux);
  if (aux >= 0 && aux <= 3) {
    gain_force = aux;
  }
  

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
  double dist   = t - x;
  double a1t2   = tdd*tau2;
  double a0t2   = xdd*tau2;
  double v1t1   = td*tau1;
  double v0t1   = xd*tau1;

  // guards against numerical drift for large tau
  //if (fabsl(dist) < 1.e-5)
  //  dist = 0.0;

  double c1 = (6.*dist + (a1t2 - a0t2)/2. - 3.*(v0t1 + v1t1));
  double c2 = (-15.*dist + (3.*a0t2 - 2.*a1t2)/2. + (8.*v0t1 + 7.*v1t1))*tau1; 
  double c3 = (10.*dist + (a1t2 - 3.*a0t2)/2. - (6.*v0t1 + 4.*v1t1))*tau2; 
  double c4 = xdd/2.;
  double c5 = xd;
  double c6 = x;
  
  *x_next   = (c1*t5 + c2*t4 + c3*t3)/tau5 + c4*t2 + c5*t1 + c6;
  *xd_next  = (5.*c1*t4 + 4*c2*t3 + 3*c3*t2)/tau5 + 2.*c4*t1 + c5;
  *xdd_next = (20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1)/tau5 + 2.*c4;

  
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

  format: every state is in curely brackets, and has keywords before every group of values

{
   "name" state_name
   "name_next" next_state_name
   "duration" movement_duration
   "manipulation_frame" ["abs" | "ref" | "bref"]
   "func_call" "...." (arbitrary name of implemented function to be called at this state)
   "pose_x" ["abs" | "rel" | "refref"] pose_x_X pose_x_Y pose_x_Z
   "pose_q" use_orient ["abs" | "rel | relref"] pose_q_Q0 pose_q_Q1 pose_q_Q2 pose_q_Q3 pose_q_Q4
   "gripper_start" ["abs" | "rel"] gripper_start_width gripper_start_force
   "gripper_end" ["abs" | "rel"] gripper_end_width gripper_end_force
   "cart_gain_x_scale" cart_gain_x_X cart_gain_x_Y cart_gain_x_Z cart_gain_xd_X cart_gain_xd_Y cart_gain_xd_Z
   "cart_gain_a_scale" cart_gain_a_A cart_gain_a_B cart_gain_a_G cart_gain_ad_A cart_gain_ad_B cart_gain_ad_G
   "cart_gain_integral" cart_gain_integral
   "ff_wrench" fx fy fz mx my mz
   "max_wrench" ["cont" | "abort" | "last"] fx_max fy_max fz_max mx_max my_max mz_max
   "controller" controller_name
   "exit_condition" ["none" | "pos" | "orient" | "pos_orient" | "force" | "moment" | "force_moment"] exit_timeout err_pos err_orient err_force err_moment
}

 ******************************************************************************/
static char state_group_names[][100]=
  {
   {"dummy"},
   {"name"},
   {"name_next"},
   {"duration"},
   {"manipulation_frame"},
   {"func_call"},
   {"pose_x"},
   {"pose_q"},
   {"gripper_start"},
   {"gripper_end"},
   {"cart_gain_x_scale"},
   {"cart_gain_a_scale"},
   {"cart_gain_integral"},
   {"force_desired"},
   {"moment_desired"},
   {"max_wrench"},
   {"controller"},
   {"exit_condition"}
  };

static int n_parms[] = {0,1,1,1,1,2,4,6,3,3,6,6,1,4,4,7,1,6};
#define MAX_BIG_STRING 5000

static int
read_state_machine(char *fname) {
  
  int    j,i,rc;
  char   string[MAX_BIG_STRING+1];
  char   default_controller_name[100];
  FILE  *in;
  int    n_read;
  StateMachineTarget sm_temp;
  int    found_start = FALSE;
  int    count = 0;
  char  *c;
  int    cr;
  char   saux[MAX_BIG_STRING+1];
  double aux;
  int    iaux;
    
  // open the file and strip all comments
  sprintf(string,"%s%s",PREFS,fname);
  in = fopen_strip(string);
  if (in == NULL) {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return FALSE;
  }

  // look for a possible endefector specification
  if (find_keyword(in,"endeffector")) {
    double m, mcm[N_CART+1],x[N_CART+1],a[N_CART+1];
    
    rc=fscanf(in,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
	      &m,&mcm[_X_],&mcm[_Y_],&mcm[_Z_],&x[_X_],&x[_Y_],&x[_Z_],&a[_A_],&a[_B_],&a[_G_]);

    if (rc == 10) { // assigne the endeffector structure
      endeff[HAND].m = m;
      for (i=1; i<=N_CART; ++i) {
	endeff[HAND].mcm[i] = mcm[i];
	endeff[HAND].x[i]   = x[i];
	endeff[HAND].a[i]   = a[i];
      }
      broadcastEndeffector(endeff);
    }
  }
  rewind(in);
  
  // look for a default controller
  if (find_keyword(in,"default_controller")) {
    rc=fscanf(in,"%s",default_controller_name);
    if (rc == 1) {
      printf("Found default controller: %s\n",default_controller_name);
    } else {
      sprintf(default_controller_name,"none");
    }
  }
  rewind(in);


  // look for a reference pose_x
  if (find_keyword(in,"reference_state_pose_x")) {
    printf("Found reference pose_x\n");
    rc=fscanf(in,"%lf %lf %lf", &reference_state_pose_x_base[_X_],
               &reference_state_pose_x_base[_Y_],&reference_state_pose_x_base[_Z_]);
  } else {
    printf("WARNING: reference pose_x defaults to current pose_x\n");
    for (i=1; i<=N_CART; ++i)
      reference_state_pose_x_base[i] = cart_state[HAND].x[i];
  }
 
  for (i=1; i<=N_CART; ++i)
    reference_state_pose_x[i] = reference_state_pose_x_base[i];
  
  rewind(in);


   // look for a reference pose_q
  if (find_keyword(in,"reference_state_pose_q")) {
    printf("Found reference pose_q\n");
    rc=fscanf(in,"%lf %lf %lf %lf", &reference_state_pose_q_base[_Q0_],&reference_state_pose_q_base[_Q1_],
	      &reference_state_pose_q_base[_Q2_],&reference_state_pose_q_base[_Q3_]);
  } else {
    printf("WARNING: reference pose_q defaults to current pose_q\n");
    for (i=1; i<=N_QUAT; ++i)
      reference_state_pose_q_base[i] = cart_orient[HAND].q[i];
  }

  quatNorm(reference_state_pose_q_base);

  for (i=1; i<=N_QUAT; ++i)
    reference_state_pose_q[i] = reference_state_pose_q_base[i];

  rewind(in);

  // look for cartesian default gains
  if (find_keyword(in,"default_cart_gain_x_scale")) {
    printf("Found default gains in x\n");
    rc=fscanf(in,"%lf %lf %lf %lf %lf %lf",
	      &default_cart_gain_x_scale[1],
	      &default_cart_gain_x_scale[2],
	      &default_cart_gain_x_scale[3],
	      &default_cart_gain_x_scale[4],
	      &default_cart_gain_x_scale[5],
	      &default_cart_gain_x_scale[6]);
  } else {
    printf("WARNING: use x gain defaults\n");
    for (i=1; i<=N_CART*2; ++i)
      default_cart_gain_x_scale[i] = 1.0;
  }

  rewind(in);

  if (find_keyword(in,"default_cart_gain_a_scale")) {
    printf("Found default gains in a\n");
    rc=fscanf(in,"%lf %lf %lf %lf %lf %lf",
	      &default_cart_gain_a_scale[1],
	      &default_cart_gain_a_scale[2],
	      &default_cart_gain_a_scale[3],
	      &default_cart_gain_a_scale[4],
	      &default_cart_gain_a_scale[5],
	      &default_cart_gain_a_scale[6]);
  } else {
    printf("WARNING: use a gain defaults\n");
    for (i=1; i<=N_CART*2; ++i)
      default_cart_gain_a_scale[i] = 1.0;
  }

  rewind(in);

  // look for a reference_pose_delta table
  n_states_pose_delta = 0;
  current_state_pose_delta = 0;
  if (find_keyword(in,"reference_state_pose_delta_table")) {
    char fname[100];
    FILE *fp=NULL;
    int  count;
    double aux;

    rc=fscanf(in,"%s",fname);
    fp = fopen_strip(fname);
    if (fp != NULL) {
      char allow_base_reference[100];

      rc = fscanf(fp,"%s %d",allow_base_reference,&allow_base_reference_frame);
      if (rc != 2 || strcmp(allow_base_reference,"allow_base_reference")!=0 ) {
	printf("Table should have >allow_base_reference = 0 or 1< in first line!\n");
	printf("Assume >allow_base_reference = 0< \n");
	allow_base_reference_frame = FALSE;
	rewind(fp);
      }
      
      while (TRUE) {
	rc = fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf",&aux,&aux,&aux,&aux,&aux,&aux,&aux);
	if (rc == 7 && n_states_pose_delta < MAX_STATES_SM) {
	  ++n_states_pose_delta;
	} else {
	  break;
	}
      }
      rewind(fp);

      rc = fscanf(fp,"%s %d",allow_base_reference,&allow_base_reference_frame);
      if (rc != 2 && strcmp(allow_base_reference,"allow_base_reference")!=0 ) {
	allow_base_reference_frame = FALSE;
	rewind(fp);
      }

      if (allow_base_reference_frame < 0 || allow_base_reference_frame > 1)
	allow_base_reference_frame = FALSE;
      
      for (i=1; i<=n_states_pose_delta; ++i) {
	rc = fscanf(fp,"%lf %lf %lf %lf %lf %lf %lf",
		    &reference_state_pose_delta_x_table[i][_X_],
		    &reference_state_pose_delta_x_table[i][_Y_],
		    &reference_state_pose_delta_x_table[i][_Z_],
		    &reference_state_pose_delta_q_table[i][_Q0_],
		    &reference_state_pose_delta_q_table[i][_Q1_],
		    &reference_state_pose_delta_q_table[i][_Q2_],
		    &reference_state_pose_delta_q_table[i][_Q3_]);
	quatNorm(reference_state_pose_delta_q_table[i]);
      }
      printf("Found table of pose perturbations with %d entries\n",n_states_pose_delta);
    } else {
      printf("reference_state_pose_delta_table with name >%s< could not be found\n",fname);
    }
    
  }
  
  rewind(in);
  
    
  // zero the number of states in state machine
  n_states_sm = 0;
  
  // read states into a string, and then parse the string
  while ((cr=fgetc(in)) != EOF) {

    if ( cr == '{' ) { // found the beginning of a state
      
      found_start = TRUE;
      count = 0;
      
    } else if ( cr == '}' && found_start ) { // a complete record was found

      found_start = FALSE;
      bzero((char *)&sm_temp,sizeof(sm_temp));  

      if (count < MAX_BIG_STRING) {
	string[count++] = '\0';

	// now parse the string

	// the name
	i = 1;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  printf("Could not find group %s\n",state_group_names[i]);
	  continue;
	} else {
	  n_read = sscanf(c,"%s",sm_temp.state_name);
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}

	// the name of the next state (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  ;
	} else {
	  n_read = sscanf(c,"%s",sm_temp.next_state_name);
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}

	// the duration
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  printf("Could not find group %s\n",state_group_names[i]);
	  continue;
	} else {
	  n_read = sscanf(c,"%lf",&sm_temp.movement_duration);
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}

	// the frame in which to manipulate (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.manipulation_frame = ABS_FRAME;
	} else {
	  n_read = sscanf(c,"%s",saux);
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	  if (strcmp(saux,"ref")==0)
	    sm_temp.manipulation_frame = REF_FRAME;
	  else if (strcmp(saux,"bref")==0)
	    sm_temp.manipulation_frame = BASE_REF_FRAME;
	  else
	    sm_temp.manipulation_frame = ABS_FRAME;	    	  
	}

	// an arbitrary function to be called at the beginning of state switch (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.function_call = NO_FUNC;
	} else {
	  n_read = sscanf(c,"%s %d",saux,&iaux);
	  if (n_read != n_parms[i]) {
	    if (n_read == 1) { // backward compatibility
	      iaux = 0;
	    } else {
	      printf("Expected %d elements, but found only %d elements  in group %s\n",
		     n_parms[i],n_read,state_group_names[i]);
	      continue;
	    }
	  }

	  sm_temp.n_function_args = iaux;

	  // read arguments of function call
	  if (iaux > 0) {
	    n_read = sscanf(c,"%s %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",saux,&iaux,
			    &sm_temp.function_args[1],
			    &sm_temp.function_args[2],
			    &sm_temp.function_args[3],
			    &sm_temp.function_args[4],
			    &sm_temp.function_args[5],			    
			    &sm_temp.function_args[6],
			    &sm_temp.function_args[7],
			    &sm_temp.function_args[8],
			    &sm_temp.function_args[9],
			    &sm_temp.function_args[10]			    
			    );
	    if (n_read != n_parms[i] + iaux) {
	      printf("Expected %d elements, but found only %d elements  in group %s\n",
		     n_parms[i]+iaux,n_read,state_group_names[i]);
	      continue;
	    }
	  }

	  // determine which function call is requested
	  if (strcmp(saux,"zero_ft")==0)
	    sm_temp.function_call = ZERO_FT;
	  else if (strcmp(saux,"next_table_delta")==0)
	    sm_temp.function_call = NEXT_TABLE_DELTA;
	  else if (strcmp(saux,"gripper")==0)
	    sm_temp.function_call = GRIPPER;
	  else if (strcmp(saux,"send_pose")==0) {
	    if (initDataCollection()) {
	      sm_temp.function_call = SEND_POSE_EST_DATA;
	    } else {
	      printf("Could not start data collection\n");	      
	      sm_temp.function_call = NO_FUNC;
	    }
	  }
	  else if (strcmp(saux,"correct_pose")==0)
	    if (initDataCollection()) {	    
	      sm_temp.function_call = RECEIVE_POSE_CORRECTION;
	    } else {
	      printf("Could not start data collection\n");
	      sm_temp.function_call = NO_FUNC;	      
	    }
	  else if (strcmp(saux,"reset_base_pose")==0)
	    sm_temp.function_call = RESET_TO_BASE_POSE;
	  else if (strcmp(saux,"add_statistics")==0)
	    sm_temp.function_call = ADD_CURRENT_TO_STATISTICS;
	  else if (strcmp(saux,"grasp_perturbation")==0) {
	    sm_temp.function_call = GRASP_PERTURBATION;
	  }
	  else
	    sm_temp.function_call = NO_FUNC;
	}
    

	// pose_x
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  printf("Could not find group %s\n",state_group_names[i]);
	  continue;
	} else {
	  n_read = sscanf(c,"%s %lf %lf %lf",
			  saux,&(sm_temp.pose_x[_X_]),&(sm_temp.pose_x[_Y_]),&(sm_temp.pose_x[_Z_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	  if (strcmp(saux,"rel")==0)
	    sm_temp.pose_x_is_relative = REL;
	  else if (strcmp(saux,"relref")==0)
	    sm_temp.pose_x_is_relative = RELREF;
	  else
	    sm_temp.pose_x_is_relative = ABS;
	}

	// pose_q (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.use_orient = FALSE;
	} else {
	  n_read = sscanf(c,"%d %s %lf %lf %lf %lf", &(sm_temp.use_orient),saux,
			  &(sm_temp.pose_q[_Q0_]),
			  &(sm_temp.pose_q[_Q1_]),
			  &(sm_temp.pose_q[_Q2_]),
			  &(sm_temp.pose_q[_Q3_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	  quatNorm(sm_temp.pose_q);
	  if (strcmp(saux,"rel")==0)
	    sm_temp.pose_q_is_relative = REL;
	  else if (strcmp(saux,"relref")==0)
	    sm_temp.pose_q_is_relative = RELREF;
	  else
	    sm_temp.pose_q_is_relative = ABS;

	  //normalize quaternion for safety
	  aux = 0.0;
	  for (j=1; j<=N_QUAT; ++j)
	    aux += sqr(sm_temp.pose_q[j]);

	  if (aux < 0.95) {
	    printf("Quaternion in group %s not normalized\n",state_group_names[i]);
	    continue;
	  }

	  for (j=1; j<=N_QUAT; ++j)
	    sm_temp.pose_q[j] /= sqrt(aux);
	}

	
	// gripper_start (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.gripper_start_active = FALSE;
	} else {
	  n_read = sscanf(c,"%s %lf %lf",saux,&(sm_temp.gripper_width_start), &(sm_temp.gripper_force_start));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	  sm_temp.gripper_start_active = TRUE;	  
	  if (strcmp(saux,"rel")==0)
	    sm_temp.gripper_width_start_is_relative = TRUE;
	  else
	    sm_temp.gripper_width_start_is_relative = FALSE;
	}


	// gripper_end
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.gripper_end_active = FALSE;	  
	} else {
	  n_read = sscanf(c,"%s %lf %lf",saux,&(sm_temp.gripper_width_end), &(sm_temp.gripper_force_end));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	  sm_temp.gripper_end_active = TRUE;	  	  
	  if (strcmp(saux,"rel")==0)
	    sm_temp.gripper_width_end_is_relative = TRUE;
	  else
	    sm_temp.gripper_width_end_is_relative = FALSE;
	}


	// gain_x (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  for (j=1; j<=N_CART; ++j) {
	    sm_temp.cart_gain_x_scale[j]  = default_cart_gain_x_scale[j];
	    sm_temp.cart_gain_xd_scale[j] = default_cart_gain_x_scale[j+N_CART];
	  }
	} else {
	  n_read = sscanf(c,"%lf %lf %lf %lf %lf %lf",
			  &(sm_temp.cart_gain_x_scale[_X_]),
			  &(sm_temp.cart_gain_x_scale[_Y_]),
			  &(sm_temp.cart_gain_x_scale[_Z_]),
			  &(sm_temp.cart_gain_xd_scale[_X_]),
			  &(sm_temp.cart_gain_xd_scale[_Y_]),
			  &(sm_temp.cart_gain_xd_scale[_Z_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}


	// gain_a (optional) 
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  for (j=1; j<=N_CART; ++j) {
	    sm_temp.cart_gain_a_scale[j]  = default_cart_gain_a_scale[j];
	    sm_temp.cart_gain_ad_scale[j] = default_cart_gain_a_scale[j+N_CART];
	  }
	} else {
	  n_read = sscanf(c,"%lf %lf %lf %lf %lf %lf",
			  &(sm_temp.cart_gain_a_scale[_X_]),
			  &(sm_temp.cart_gain_a_scale[_Y_]),
			  &(sm_temp.cart_gain_a_scale[_Z_]),
			  &(sm_temp.cart_gain_ad_scale[_X_]),
			  &(sm_temp.cart_gain_ad_scale[_Y_]),
			  &(sm_temp.cart_gain_ad_scale[_Z_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",
		   n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}


	// the integral gains (optinal)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.cart_gain_integral = 0.0;
	} else {
	  n_read = sscanf(c,"%lf",&sm_temp.cart_gain_integral);
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}

	// simulator does not like high integral gains
	if (!real_robot_flag)
	  if (sm_temp.cart_gain_integral > 0.01)
	    sm_temp.cart_gain_integral = 0.01;

	// force_desired (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.force_des_gain = 0.0;
	} else {
	  n_read = sscanf(c,"%lf %lf %lf %lf",
			  &(sm_temp.force_des_gain),
			  &(sm_temp.force_des[_X_]),
			  &(sm_temp.force_des[_Y_]),
			  &(sm_temp.force_des[_Z_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}


	// moment_desired (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.moment_des_gain = 0.0;
	} else {
	  n_read = sscanf(c,"%lf %lf %lf %lf",
			  &(sm_temp.moment_des_gain),			  
			  &(sm_temp.moment_des[_A_]),
			  &(sm_temp.moment_des[_B_]),
			  &(sm_temp.moment_des[_G_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}


	// max_wrench (optional)
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  for (j=1; j<=N_CART*2; ++j)
	    sm_temp.max_wrench[j] = 1000;
	  sm_temp.ft_exception_action = CONT;
	} else {
	  n_read = sscanf(c,"%s %lf %lf %lf %lf %lf %lf",saux,
			  &(sm_temp.max_wrench[_X_]),
			  &(sm_temp.max_wrench[_Y_]),
			  &(sm_temp.max_wrench[_Z_]),
			  &(sm_temp.max_wrench[N_CART+_A_]),
			  &(sm_temp.max_wrench[N_CART+_B_]),
			  &(sm_temp.max_wrench[N_CART+_G_]));
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }

	  if (strcmp(saux,"cont")==0)
	    sm_temp.ft_exception_action = CONT;
	  else if (strcmp(saux,"last")==0)
	    sm_temp.ft_exception_action = LAST;
	  else
	    sm_temp.ft_exception_action = ABORT;
	  
	}


	// the controller
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL && strcmp(default_controller_name,"none")==0) {
	  printf("Could not find group %s\n",state_group_names[i]);
	  continue;
	} else if (c == NULL) {
	  strcpy(sm_temp.controller_name,default_controller_name);
	} else {
	  n_read = sscanf(c,"%s",sm_temp.controller_name);
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }
	}

	// the exit condition
	++i;
	c = find_keyword_in_string(string,state_group_names[i]);
	if (c == NULL) {
	  sm_temp.exit_condition = NO_EXIT;
	} else {
	  n_read = sscanf(c,"%s %lf %lf %lf %lf %lf",
			  saux,
			  &sm_temp.exit_timeout,
			  &sm_temp.err_pos,
			  &sm_temp.err_orient,
			  &sm_temp.err_force,
			  &sm_temp.err_moment			  			  
			  );
	  if (n_read != n_parms[i]) {
	    printf("Expected %d elements, but found only %d elements  in group %s\n",n_parms[i],n_read,state_group_names[i]);
	    continue;
	  }

	  //   "exit_condition" ["none" | "pos" | "orient" | "pos_orient" | "force" | "moment" | "force_moment" | "func_success" ] tiime_out err_pos err_orient err_force err_moment

	  if (strcmp(saux,"pos")==0)
	    sm_temp.exit_condition = POS_EXIT;
	  else if (strcmp(saux,"orient")==0)
	    sm_temp.exit_condition = ORIENT_EXIT;
	  else if (strcmp(saux,"pos_orient")==0)
	    sm_temp.exit_condition = POS_ORIENT_EXIT;
	  else if (strcmp(saux,"force")==0)
	    sm_temp.exit_condition = FORCE_EXIT;
	  else if (strcmp(saux,"moment")==0)
	    sm_temp.exit_condition = MOMENT_EXIT;
	  else if (strcmp(saux,"force_moment")==0)
	    sm_temp.exit_condition = FORCE_MOMENT_EXIT;
	  else if (strcmp(saux,"table")==0)
	    sm_temp.exit_condition = EXIT_AFTER_TABLE;
	  else if (strcmp(saux,"func_success")==0)
	    sm_temp.exit_condition = EXIT_AFTER_FUNCTION_CALL_SUCCESS;
	  else 
	    sm_temp.exit_condition = NO_EXIT;


	  if (sm_temp.exit_timeout < 0)
	    sm_temp.exit_timeout = 0;
	  
	}

	// finish up
	if (n_states_sm < MAX_STATES_SM) {
	  Matrix mx,mxd,ma,mad; // need to store pointers temporarily

	  ++n_states_sm;

	  mx  = targets_sm[n_states_sm].cart_gain_x_scale_matrix;
	  mxd = targets_sm[n_states_sm].cart_gain_xd_scale_matrix;
	  ma  = targets_sm[n_states_sm].cart_gain_a_scale_matrix;
	  mad = targets_sm[n_states_sm].cart_gain_ad_scale_matrix;

	  targets_sm[n_states_sm] = sm_temp;

	  targets_sm[n_states_sm].cart_gain_x_scale_matrix = mx;
	  targets_sm[n_states_sm].cart_gain_xd_scale_matrix = mxd;
	  targets_sm[n_states_sm].cart_gain_a_scale_matrix = ma;
	  targets_sm[n_states_sm].cart_gain_ad_scale_matrix = mad;
	  
	} else {
	  // should be unlikely to happen ever
	  printf("Error: ran out of memory for state machine targets\n");
	  return FALSE;
	}


      } else {  // should not be possible: too many characters witout a closing curly bracket

	found_start = FALSE;

      }

    } else {

      if (found_start && count < MAX_BIG_STRING) {
	string[count++] = cr;
      } else {  // should not be possible: too many characters witout a closing curly bracket
	found_start = FALSE;
      }
      
    }

  }

  // got through all states and fill in a next_state_id in case a next_state exists
  found_recurrance = FALSE;
  for (i=1; i<=n_states_sm; ++i) {
    if (strcmp(targets_sm[i].next_state_name,"") != 0) {
      for (j=1; j<=n_states_sm; ++j) {
	if (strcmp(targets_sm[i].next_state_name,targets_sm[j].state_name) == 0) {
	  targets_sm[i].next_state_id = j;
	  found_recurrance = TRUE;
	}
      }
    }
  }
      

  // all done parsing
  
  printf("Read %d states for the state machine\n",n_states_sm);
  
  fclose(in);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  save_statistics_matrix
\date  Nov 2020
   
\remarks 

       just saves relevant statistics matrix to file in ascii, where file
       name includes date/time of creation

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void 
save_statistics_matrix(void)
{
  int i,j;
  FILE *fp;
  char fname[200];
  time_t rawtime;
  struct tm *tptr;
  char   string[200]="";
  
  get_string("Additional string in filename?",string,string);
  
  time(&rawtime);
  tptr = localtime(&rawtime);
  sprintf(fname,"stat_matrix_%s_%d-%d-%d-%d:%d:%d.txt",string,tptr->tm_year+1900,tptr->tm_mon+1,tptr->tm_mday,tptr->tm_hour,tptr->tm_min,tptr->tm_sec);

  // open file
  fp = fopen(fname,"w");
  if (fp == NULL) {
    printf("Cannot open file %s\n",fname);
    return;
  }

  for (i=1; i<=n_states_pose_delta; ++i) {
    for (j=1; j<=8*(N_CART+N_QUAT); ++j) {
      fprintf(fp,"%f ",statistics_matrix[i][j]);
      if (j%7 == 0)
	fprintf(fp,"    ");
    }
    fprintf(fp,"\n");
  }

  fclose(fp);

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
\note  print_sm_stats
\date  Dec. 2020
   
\remarks 

       prints current statistics/info of state machine state

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

  none

 ******************************************************************************/
static void 
print_sm_stats(void)
{
  int i;

  printf("StateName             : %s\n",current_target_sm.state_name);

  printf("Reference State Pos   : %f %f %f\n",
	 reference_state_pose_x[_X_],
	 reference_state_pose_x[_Y_],
	 reference_state_pose_x[_Z_]);

  printf("Reference State Orient: %f %f %f %f\n",
	 reference_state_pose_q[_Q0_],
	 reference_state_pose_q[_Q1_],
	 reference_state_pose_q[_Q2_],
	 reference_state_pose_q[_Q3_]);	 

  printf("Target Pos Error      : %f (%f %f %f)\n",
	 pos_error, pos_error_vector[_X_], pos_error_vector[_Y_], pos_error_vector[_Z_]);

  printf("Target Orient Error   : %f (%f %f %f %f)\n",
	 orient_error, orient_error_quat[_Q0_], orient_error_quat[_Q1_],
	 orient_error_quat[_Q2_], orient_error_quat[_Q3_]);

  printf("Tracking Pos Stats    : %f < %f < %f\n",min_pos_tracking_error,
	 mean_pos_tracking_error, max_pos_tracking_error);

  printf("Tracking Orient Stats : %f < %f < %f\n",min_orient_tracking_error,
	 mean_orient_tracking_error, max_orient_tracking_error);

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
 nice smooth unit norm quaternions.

 Note: the start quaternion is needed, and thus, this implementation depends
 at every time step on the start and target quaternion

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          q_start  : the start orientation as quaterion
 \param[in]          q_target : the target orientation as quaterion
 \param[in,out]      s        : vector of s,sd,sdd which does min jerk from 1 to zero
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[out]         q_next   : the next state quaternion with all derivatives

 ******************************************************************************/
static int
min_jerk_next_step_quat (SL_quat q_start, SL_quat q_target, double *s,
			 double t_togo, double dt, SL_quat *q_next)

{
  int    i,j;
  double theta; // angle between start and target quaternion
  double aux;
  double ridge = 1e-10;
  SL_quat q_last;

  // adjust the target to be in the same solution space as the start
  aux = vec_mult_inner_size(q_start.q,q_target.q,N_QUAT);
  if (aux < 0) {
    //printf("aux = %f\n",aux);
    //print_vec_size("q_start",q_start.q,4);
    //print_vec_size("target",q_target.q,4);  
    vec_mult_scalar_size(q_target.q,N_QUAT,-1.0,q_target.q);
  }

  // make sure numerical issues of quaternion inner product cannot bother us
  aux = vec_mult_inner_size(q_start.q,q_target.q,N_QUAT);
  if (aux > 1)
    aux = 1;
  else if (aux < -1)
    aux = -1;
  theta = acos( aux );

  // start and target are numerically essentially identical
  if (fabs(theta) <= 1.e-6){
    for (i=1; i<=N_QUAT; ++i) {
      q_next->q[i] = q_start.q[i];
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

  // store last quaternion
  q_last = *q_next;

  // interpolate
  for (i=1; i<=N_QUAT; ++i) {
     q_next->q[i] = sin(theta*(1.0-s[1]))/aux*q_start.q[i] + sin(theta*s[1])/aux*q_target.q[i];
  }

  // fill in derivatives from numerical differentiation
  for (j=1; j<=N_QUAT; ++j){
    q_next->qd[j]  =  (q_next->q[j] - q_last.q[j]) / dt;
    q_next->qdd[j] =  (q_next->qd[j] - q_last.qd[j]) / dt;
  }

  //  fprintf(fp,"%f %f %f   %f %f %f %f\n",s[1],s[2],s[3],q_next->q[1],q_next->q[2],q_next->q[3],q_next->q[4]);
  
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  min_jerk_next_step_quat_new
\date  March 2021
   
\remarks 

 min jerk next step spline in quaternions. This is a first principles
 correct 5th order orientation spline, exploiting the log() and exp()
 transforms of quaternions. There is no internal state, and the spline can
 start from any pos/vel/acc to any target in pos/vel/acc.

 Thanks to Giovanni Sutanto for pointing out the right approach.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          q        : the current quaternion with all derivatives
 \param[in]          q_target : the target quaternion with all derivatives
 \param[in]          t_togo   : time to go until target is reached
 \param[in]          dt       : time increment
 \param[out]         q_next   : the next state quaternion with all derivatives

 ******************************************************************************/
static int
min_jerk_next_step_quat_new (SL_quat q, SL_quat q_target,
			     double t_togo, double dt, SL_quat *q_next)

{
  int    i,j;
  double aux;
  double t1,t2,t3,t4,t5;
  double tau,tau1,tau2,tau3,tau4,tau5;

  // a safety check
  if (dt > t_togo || dt <= 0) {
    return FALSE;
  }

  // the target quaternion should be in the same solution space as the start
  // quaternion (TBD: this issue may need to be checked carefully for general validity)
  aux = vec_mult_inner_size(q.q,q_target.q,N_QUAT);
  if (aux < 0) {
    vec_mult_scalar_size(q_target.q,N_QUAT,-1.0,q_target.q);
  }

  // highly efficient precompuation of time constants
  t1 = dt;
  t2 = t1 * dt;
  t3 = t2 * dt;
  t4 = t3 * dt;
  t5 = t4 * dt;

  tau  = tau1 = t_togo;
  tau2 = tau1 * tau;
  tau3 = tau2 * tau;
  tau4 = tau3 * tau;
  tau5 = tau4 * tau;

  // calculate the spline constants: note that the spline is in angular (tangent) space, not
  // quaternion space, and thus all relevant quanities become 3D vectors
  double q_rel[N_QUAT+1];
  double dist[N_CART+1];

  // the distance between q and q_target in tangent space
  quatRelative(q.q, q_target.q, q_rel);
  quatLog(q_rel,dist);

  // fill in angular derivatives to quaternions
  quatToAngularDerivatives(&q);
  quatToAngularDerivatives(&q_target);  
  
  double a1t2[N_CART+1];
  double a0t2[N_CART+1];
  double v1t1[N_CART+1];
  double v0t1[N_CART+1];

  for (i=1; i<=N_CART; ++i) {
    a1t2[i]   = q_target.add[i]*tau2;
    a0t2[i]   = q.add[i]*tau2;
    v1t1[i]   = q_target.ad[i]*tau1;
    v0t1[i]   = q.ad[i]*tau1;
  }

  double c1[N_CART+1];
  double c2[N_CART+1];
  double c3[N_CART+1];
  double c4[N_CART+1];
  double c5[N_CART+1];

  for (i=1; i<=N_CART; ++i) {
    c1[i] = (6.*dist[i] + (a1t2[i] - a0t2[i])/2. - 3.*(v0t1[i] + v1t1[i]));
    c2[i] = (-15.*dist[i] + (3.*a0t2[i] - 2.*a1t2[i])/2. + (8.*v0t1[i] + 7.*v1t1[i]))*tau1; 
    c3[i] = (10.*dist[i] + (a1t2[i] - 3.*a0t2[i])/2. - (6.*v0t1[i] + 4.*v1t1[i]))*tau2; 
    c4[i] = q.add[i]/2.;
    c5[i] = q.ad[i];
  }

  double delta_a_next[N_CART+1];
  double ad_next[N_CART+1];
  double add_next[N_CART+1];

  for (i=1; i<=N_CART; ++i) {
    delta_a_next[i]   = (c1[i]*t5 + c2[i]*t4 + c3[i]*t3)/tau5 + c4[i]*t2 + c5[i]*t1;
    q_next->ad[i] = ad_next[i]  = (5.*c1[i]*t4 + 4*c2[i]*t3 + 3*c3[i]*t2)/tau5 + 2.*c4[i]*t1 + c5[i];
    q_next->add[i] = add_next[i] = (20.*c1[i]*t3 + 12.*c2[i]*t2 + 6.*c3[i]*t1)/tau5 + 2.*c4[i];
  }

  // postprocess into quaternions
  quatExp(delta_a_next,q_rel);
  quatMult(q.q,q_rel,q_next->q);
  if (q_next->q[_Q0_] < 0)
    for (i=1; i<=N_QUAT; ++i)
      q_next->q[i] *= -1.0;

  quatDerivatives(q_next);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  assignCurrentSMTarget
\date  November 2019
   
\remarks 

 The new state machine state is assigned the the relevant current structures.
 Most importantly, reference frame issues are resolved, such that the resulting
 information is in base coordinates.
 

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]          smt      : state machine target
 \param[in]          ref_x    : reference position
 \param[in]          ref_q    : reference orientation
 \param[out]         ct       : cartesian target position
 \param[out]         cto      : cartesian target orientation
 \param[out]         smc      : the current state machine state

 ******************************************************************************/
static void
assignCurrentSMTarget(StateMachineTarget smt,
		      double *ref_x,
		      double *ref_q,
		      SL_Cstate *ct,
		      SL_quat   *cto,
		      StateMachineTarget *smc)
{
  int i,j,r;
  MY_MATRIX(R,1,N_CART,1,N_CART);
  SL_quat temp_q;
  double ref_q_adj[N_QUAT+1];

  // to get started, simply copy the target to the current target, and this what needs
  // to be fixed.
  *smc = smt;
  if (smt.manipulation_frame == BASE_REF_FRAME)
    smc->manipulation_frame = REF_FRAME;

  // the rotation matrix relative to the reference frame
  // assign to temp quaternion structure
  for (r=1; r<=N_QUAT; ++r)
    temp_q.q[r] = ref_q[r];

  // compute rotation matrix
  quatToRotMatInv(&temp_q,R);

  // assign the target position
  if (smc->pose_x_is_relative == REL && smc->manipulation_frame == ABS_FRAME) {
    for (i=1; i<=N_CART; ++i) {
      ct[HAND].x[i] += smc->pose_x[i];
    }
  } else if (smc->pose_x_is_relative == ABS && smc->manipulation_frame == REF_FRAME) {
    mat_vec_mult_size(R,N_CART,N_CART,smt.pose_x,N_CART,smc->pose_x);
    for (i=1; i<=N_CART; ++i) {
      ct[HAND].x[i] = ref_x[i] + smc->pose_x[i];
      if (apply_grasp_perturbation) {
	ct[HAND].x[i] += grasp_perturbation_x[i];
      }
    }
  } else if (smc->pose_x_is_relative == REL && smc->manipulation_frame == REF_FRAME) {
    mat_vec_mult_size(R,N_CART,N_CART,smt.pose_x,N_CART,smc->pose_x);
    for (i=1; i<=N_CART; ++i) {
      ct[HAND].x[i] += smc->pose_x[i];
    }
  } else { // absolute pose info just needs easy assignment
    for (i=1; i<=N_CART; ++i) {
      ct[HAND].x[i] = smc->pose_x[i];
    }
  }

  //print_vec_size("ctarget",ct[HAND].x,N_CART);
  // print_vec_size("ctarget",ct[HAND].xd,N_CART);
  //print_vec_size("ctarget",ct[HAND].xdd,N_CART);
  
  // assign the target orientation if needed
  if (smc->use_orient) {
    
    for (i=1; i<=N_CART; ++i)
      stats[N_CART+i] = 1;

    if (smc->pose_q_is_relative == REL && smc->manipulation_frame == ABS_FRAME) {	
      //print_vec_size("before",cto[HAND].q,4);
      quatMult(cto[HAND].q,smc->pose_q,cto[HAND].q);
      //print_vec_size("after",cto[HAND].q,4);
    } else if (smc->pose_q_is_relative == ABS && smc->manipulation_frame == REF_FRAME) {	
      //print_vec_size("before",cto[HAND].q,4);
      //print_vec_size("ref_state_pose",ref_q,4);
      mat_vec_mult_size(R,N_CART,N_CART,&(smt.pose_q[_Q0_]),N_CART,&(smc->pose_q[_Q0_]));
      if (apply_grasp_perturbation) {
	quatMult(ref_q,grasp_perturbation_q,ref_q_adj);	
      } else {
	for (i=1; i<=N_QUAT; ++i)
	  ref_q_adj[i] = ref_q[i];
      }
      quatMult(ref_q_adj,smc->pose_q,cto[HAND].q);
      //print_vec_size("after",cto[HAND].q,4);
    } else if (smc->pose_q_is_relative == REL && smc->manipulation_frame == REF_FRAME) {	
      //print_vec_size("before",cto[HAND].q,4);
      //print_vec_size("desired change",smt.pose_q,4);
      mat_vec_mult_size(R,N_CART,N_CART,&(smt.pose_q[_Q0_]),N_CART,&(smc->pose_q[_Q0_]));
      //print_vec_size("rotated desired change",smc->pose_q,4);      
      quatMult(cto[HAND].q,smc->pose_q,cto[HAND].q);
      //print_vec_size("after",cto[HAND].q,4);
    } else {
      for (i=1; i<=N_QUAT; ++i) {
	cto[HAND].q[i] = smc->pose_q[i];
      }
    }

  } else { // no orientation
    
    for (i=1; i<=N_CART; ++i)
      stats[N_CART+i] = 0;
    
  }

  // print_vec_size("ctarget_orient",cto[HAND].q,N_QUAT);

  // by default, the PD gains just go into a diagonal matrix
  for (i=1; i<=N_CART; ++i) {
    for (j=1; j<=N_CART; ++j) {
      if ( i == j ) {
	smc->cart_gain_x_scale_matrix[i][j] = smc->cart_gain_x_scale[i];
	smc->cart_gain_xd_scale_matrix[i][j] = smc->cart_gain_xd_scale[i];
	smc->cart_gain_a_scale_matrix[i][j] = smc->cart_gain_a_scale[i];
	smc->cart_gain_ad_scale_matrix[i][j] = smc->cart_gain_ad_scale[i];
      } else {
	smc->cart_gain_x_scale_matrix[i][j] = 0.0;
	smc->cart_gain_xd_scale_matrix[i][j] = 0.0;
	smc->cart_gain_a_scale_matrix[i][j] = 0.0;
	smc->cart_gain_ad_scale_matrix[i][j] = 0.0;
      }
    }
  }

  // convert diag control gains from local to global if the state of the state
  // machine is in a special  reference frame. Also adjust the desired and
  // max wrench
  if (smc->manipulation_frame == REF_FRAME) { // gains are in reference frame
    MY_MATRIX(T,1,N_CART,1,N_CART);
    
    // rotate gain matrix appropriately
    mat_mult(R,smc->cart_gain_x_scale_matrix,T);
    mat_mult_normal_transpose(T,R,smc->cart_gain_x_scale_matrix);

    mat_mult(R,smc->cart_gain_xd_scale_matrix,T);
    mat_mult_normal_transpose(T,R,smc->cart_gain_xd_scale_matrix);

    mat_mult(R,smc->cart_gain_a_scale_matrix,T);
    mat_mult_normal_transpose(T,R,smc->cart_gain_a_scale_matrix);

    mat_mult(R,smc->cart_gain_ad_scale_matrix,T);
    mat_mult_normal_transpose(T,R,smc->cart_gain_ad_scale_matrix);

    /*
    print_mat("x-scale",smc->cart_gain_x_scale_matrix);
    print_mat("xd-scale",smc->cart_gain_xd_scale_matrix);
    print_mat("a-scale",smc->cart_gain_a_scale_matrix);
    print_mat("ad-scale",smc->cart_gain_ad_scale_matrix);
    */

    // rotate force/torque values
    mat_vec_mult_size(R,N_CART,N_CART,smt.force_des,N_CART,smc->force_des);
    mat_vec_mult_size(R,N_CART,N_CART,smt.moment_des,N_CART,smc->moment_des);
    mat_vec_mult_size(R,N_CART,N_CART,smt.max_wrench,N_CART,smc->max_wrench);
    mat_vec_mult_size(R,N_CART,N_CART,&(smt.max_wrench[_Z_]),N_CART,&(smc->max_wrench[_Z_]));
    for (i=1; i<=2*N_CART; ++i)
      smc->max_wrench[i] = fabs(smc->max_wrench[i]);

  } 
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  functionCall
\date  Oct 2020
   
\remarks 

 A switch statement to manage function calls and a possible success feedback.
 The first call at the initialization phasee of a sm state sets the initial_call
 flag to TRUE. During execution of the state, at every timestep, the functionCalls
 is called with initial_call = FALSE, such that only the success termination
 of the function call is checked.
 

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output


 \param[in]          id           : functionCalls ID as in ENUM statement
 \param[in]          initial_call : TRUE/FALSE for first call at state
                                    initialization.
 \param[out]         success      : TRUE or FALSE for function_call terminated
                                    successfully

 returns FALSE when error happens, otherwise TRUE

 ******************************************************************************/
static int
functionCall(int id, int initial_call, int *success) 
{
  static int count =0;
  char   msg[1000];

  // counts how often this function was called after the last initial_call=TRUE
  if (initial_call)
    count=0;
  else
    ++count;

  switch (id) {
    
    // zero the force torque cell 
  case ZERO_FT:
    if  (initial_call) {
      sprintf(msg,"Calibrate F/T offsets\n");
      logMsg(msg,0,0,0,0,0,0);      
      sendCalibrateFTCommand();
      *success = FALSE;
    } else {
      // calibration takes  100 ticks, plus extra safety ticks
      if (count > 100+100)
	*success = TRUE;
      else
	*success = FALSE;
    }
    break;

    // assign a new delta pose perturbration    
  case NEXT_TABLE_DELTA:
    if  (initial_call) {

      if (current_state_pose_delta < n_states_pose_delta) {
	++current_state_pose_delta;
	sprintf(msg,"\nRunning perturbation %d\n",current_state_pose_delta);
	logMsg(msg,0,0,0,0,0,0);
	for (int i=1; i<=N_CART; ++i) {
	  reference_state_pose_x[i] = reference_state_pose_x_base[i] +
	    reference_state_pose_delta_x_table[current_state_pose_delta][i];
	}
	quatMult(reference_state_pose_q_base,reference_state_pose_delta_q_table[current_state_pose_delta],
		 reference_state_pose_q);
	quatNorm(reference_state_pose_q);
	*success = TRUE;
      } else {
	*success = FALSE;	
	return FALSE;
      }

    } else {
      *success = TRUE;      
    }
    break;

    // generarte a grasp perturbation: this is done by simulating a reference pose perturbation,
    // i.e., the reference pose is transformed by the grasp perturbation pose. The grasp perturbation
    // is in robot base coordinates, not any reference coordinate system
  case GRASP_PERTURBATION:
    if  (initial_call) {

      // assign grasp perturbation to global variable, and set flag to true for applying
      // the purtabation in target assignment
      grasp_perturbation_x[_X_] = targets_sm[current_state_sm].function_args[1];
      grasp_perturbation_x[_Y_] = targets_sm[current_state_sm].function_args[2];
      grasp_perturbation_x[_Z_] = targets_sm[current_state_sm].function_args[3];
      grasp_perturbation_q[_Q0_] = targets_sm[current_state_sm].function_args[4];
      grasp_perturbation_q[_Q1_] = targets_sm[current_state_sm].function_args[5];
      grasp_perturbation_q[_Q2_] = targets_sm[current_state_sm].function_args[6];
      grasp_perturbation_q[_Q3_] = targets_sm[current_state_sm].function_args[7];
      apply_grasp_perturbation = TRUE;
      
      *success = TRUE;	
      
    } else {
      apply_grasp_perturbation = FALSE;    
      *success = TRUE;      
    }
    break;

    // gripper motion: move or grasp
  case GRIPPER:
    if  (initial_call) {
      double des_gripper_width = current_target_sm.function_args[1];
      double des_gripper_width_tolerance = current_target_sm.function_args[2];
      double des_gripper_speed = current_target_sm.function_args[3];
      double des_gripper_force = current_target_sm.function_args[4];

      // give move command to gripper to desired position if width is larger than current width
      if (des_gripper_width > misc_sensor[G_WIDTH] || des_gripper_force == 0) {
	
	sendGripperMoveCommand(des_gripper_width,des_gripper_speed);
	
      } else { // or close gripper with force control otherwise
	
	sendGripperGraspCommand(des_gripper_width,
				des_gripper_speed,
				des_gripper_force,
				des_gripper_width_tolerance,
				des_gripper_width_tolerance);		
      }
      *success = FALSE;
    } else {
      // 10 safety tick before allowing success -- some communication delay may exist
      if (count > 100 && misc_sensor[G_MOTION] == 0.0) {
	*success = TRUE;
      } else
	*success = FALSE;
    }
    break;
    
  case SEND_POSE_EST_DATA:
    if  (initial_call) {
      triggerDataCollection();
      *success = FALSE;      
    } else {
      if (checkDataCollection()) {
	*success = TRUE;
	++global_sample_id;
      } else
	*success = FALSE;
    }
    break;
    
  case  RECEIVE_POSE_CORRECTION:
    if  (initial_call) {
      
      // for logging, all is added to the statistics matrix
      int count = 0;
      
      // the base reference pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_x_base[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_q_base[i];
      
      // the perturbed reference pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_x[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_q[i];
      
      triggerPoseDeltaPrediction();
      *success = FALSE;
      
    } else {
      if (checkPoseDeltaPrediction()) {

	extern double predicted_reference_state_pose_delta_x_inverse[];
	extern double predicted_reference_state_pose_delta_q_inverse[];
	extern double reference_state_pose_delta_x_adjusted[];
	extern double reference_state_pose_delta_q_adjusted[];
	
	
	*success = TRUE;
	++global_sample_id;

	int count = 2*(N_CART+N_QUAT);;

	// the adjusted delta pose
	for (int i=1; i<=N_CART; ++i)
	  statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_delta_x_adjusted[i];
	for (int i=1; i<=N_QUAT; ++i)
	  statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_delta_q_adjusted[i];
	
	// the current desired endeffector pose target
	for (int i=1; i<=N_CART; ++i)
	  statistics_matrix[current_state_pose_delta][++count] = ctarget[HAND].x[i];
	for (int i=1; i<=N_QUAT; ++i)
	  statistics_matrix[current_state_pose_delta][++count] = ctarget_orient[HAND].q[i];
	
	// the current endeffector pose
	for (int i=1; i<=N_CART; ++i)
	  statistics_matrix[current_state_pose_delta][++count] = cart_state[HAND].x[i];
	for (int i=1; i<=N_QUAT; ++i)
	  statistics_matrix[current_state_pose_delta][++count] = cart_orient[HAND].q[i];

      } else
	*success = FALSE;
    }
    break;
    
    // simply resets the reference pose to the base reference pose
  case RESET_TO_BASE_POSE:

    if  (initial_call) {
    
      extern double predicted_reference_state_pose_delta_x_inverse[];
      extern double predicted_reference_state_pose_delta_q_inverse[];
      
      // for logging, all is added to the statistics matrix
      int count = 0;
      
      // the base reference pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_x_base[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_q_base[i];
      
      // the perturbed reference pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_x[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = reference_state_pose_q[i];
      
      // the adjusted delta pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = 0; // not relevant
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = 0; // not relevant
      
      // the current desired endeffector pose target
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = ctarget[HAND].x[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = ctarget_orient[HAND].q[i];
      
      // the current endeffector pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = cart_state[HAND].x[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = cart_orient[HAND].q[i];
      
      // the pose correction from DL: this is just faked here and put into right variables
      for (int i=1; i<=N_CART; ++i)
	predicted_reference_state_pose_delta_x_inverse[i] = -reference_state_pose_delta_x_table[current_state_pose_delta][i];
      for (int i=1; i<=N_QUAT; ++i)
	if (i != _Q0_)
	  predicted_reference_state_pose_delta_q_inverse[i] = -reference_state_pose_delta_q_table[current_state_pose_delta][i];
	else
	  predicted_reference_state_pose_delta_q_inverse[i] = reference_state_pose_delta_q_table[current_state_pose_delta][i];	  


      // reset to base pose
      for (int i=1; i<=N_CART; ++i) {
	reference_state_pose_x[i] = reference_state_pose_x_base[i];
      }
      for (int i=1; i<=N_QUAT; ++i) {
	reference_state_pose_q[i] = reference_state_pose_q_base[i];
      }
      *success = TRUE;

    } else {
      *success = TRUE;      
    }
    break;

    // adds current target and actual pose to statistics matrix
  case ADD_CURRENT_TO_STATISTICS:

    if  (initial_call) {
      extern double predicted_reference_state_pose_delta_x_inverse[];
      extern double predicted_reference_state_pose_delta_q_inverse[];
      
      *success = TRUE;

      // for logging, all is added to the statistics matrix
      int count = 5*(N_CART+N_QUAT);
      
      // the pose correction from DL
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = predicted_reference_state_pose_delta_x_inverse[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = predicted_reference_state_pose_delta_q_inverse[i];	
	
      // the current desired endeffector pose target
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = ctarget[HAND].x[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = ctarget_orient[HAND].q[i];
      
      // the current endeffector pose
      for (int i=1; i<=N_CART; ++i)
	statistics_matrix[current_state_pose_delta][++count] = cart_state[HAND].x[i];
      for (int i=1; i<=N_QUAT; ++i)
	statistics_matrix[current_state_pose_delta][++count] = cart_orient[HAND].q[i];
      
    } else {
      *success = TRUE;      
    }
    break;

  default: // this is NO_FUNC by default
    *success = TRUE;
  }

  return TRUE;
}
