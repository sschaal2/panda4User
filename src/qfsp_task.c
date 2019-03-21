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
   INIT_MOVE_TO_TARGET,			 
   MOVE_TO_TARGET,
   IDLE
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
static double     corient_error[N_ENDEFFS*3+1];
static int        stats[N_ENDEFFS*6+1];
static SL_DJstate target[N_DOFS+1];
static SL_DJstate last_target[N_DOFS+1];
static int        firsttime = TRUE;
static double     start_time     = 0;
static double     gain           = 250;
static double     gain_orient    = 40;

static SL_Cstate  ball_state;

static double controller_gain_th[N_DOFS+1];
static double controller_gain_thd[N_DOFS+1];
static double controller_gain_int[N_DOFS+1];

static int    state_machine_state = INIT_MOVE_TO_TARGET;

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

  // go to a save posture 
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  bzero((char *)&(last_target[1]),N_DOFS*sizeof(last_target[1]));
  for (i=1; i<=N_DOFS; ++i) {
    target[i] = joint_default_state[i];
    last_target[i] = joint_default_state[i];
  }
  target[J4].th -= 0.2;
  target[J6].th += 0.2;

  if (!go_target_wait_ID(target))
    return FALSE;

  // initialize the cartesian control
  bzero((char *)&cdes,sizeof(cdes));
  bzero((char *)&cdes_orient,sizeof(cdes_orient));
  
  for (j=1; j<=N_CART; ++j) {
    
    // we do full pose control
    stats[j] = 1;
    stats[j + N_CART] = 1;
    
    // set cartesian target and current desired to current state to be safe
    cdes[HAND].x[j] = ctarget[HAND].x[j] = cart_des_state[HAND].x[j];
  }
  
  for (j=1; j<=N_QUAT; ++j) {
    cdes_orient[HAND].q[j] = ctarget_orient[HAND].q[j] = cart_des_orient[HAND].q[j];
  }
  
  // teach target orientation
  ans = FALSE;
  get_int("Teach target pose from current pose?",ans,&ans);
  if (ans)
    teach_target_pose();

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
  state_machine_state = INIT_MOVE_TO_TARGET;
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


  switch (state_machine_state) {

  case INIT_MOVE_TO_TARGET:
    time_to_go = 2;
    state_machine_state = MOVE_TO_TARGET;
    // break; // intentionally no break

  case MOVE_TO_TARGET:
    
    // plan the next step of hand  with min jerk
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
    time_to_go -= time_step;
    if (time_to_go < 0) {
      time_to_go = 0;
      state_machine_state = IDLE;
    }

    break;

  case IDLE:
    break;

  default:
    break;

  }
  

  // compute orientation error term for quaterion feedback control
  quatErrorVector(ctarget_orient[1].q,cart_orient[1].q,corient_error);
  
  // prepare the impdance controller, i.e., compute operational
  // space force command 
  for (j= _X_; j<= _Z_; ++j) {
    if (stats[j]) {
      cref[j] =
	(cdes[HAND].xd[j]  - cart_state[HAND].xd[j]) * 2.*sqrt(gain) +
	(cdes[HAND].x[j]  - cart_state[HAND].x[j]) * gain;
    }
  }

  for (j= _A_; j<= _G_ ; ++j) { /* orientation */
    if (stats[N_CART + j]) {
      cref[N_CART + j] = 
	(ctarget_orient[HAND].ad[j] - cart_orient[HAND].ad[j]) *0.025 * 2.0 * sqrt(gain_orient) - 
	corient_error[j] * gain_orient; 
    }
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
      O[j][i] = O[i][j];
    }
  }

  /* compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS; ++i) {
    double fac=0.2;
    e[i] = 
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd;
  }
  mat_vec_mult(O,e,en);
  //print_mat("O",O);
  //getchar();

  /* return this as a PD command in uff */
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

