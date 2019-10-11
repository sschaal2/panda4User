/*============================================================================
==============================================================================
                      
                              cilantro_track_task.c
 
==============================================================================
Remarks:

      sekeleton to create the sample task

============================================================================*/

// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"
#include "SL_unix_common.h"

// defines

// local variables
static double start_time = 0.0;
static double freq;
static double amp;
static SL_DJstate  target[N_DOFS+1];

static SL_Cstate   cart_base_state;
static int         stats[N_ENDEFFS*6+1];
static SL_quat     cdes_orient[N_ENDEFFS+1];
static SL_Cstate   cdes[N_ENDEFFS+1];

static double cart_gain_x_scale[N_CART+1]; // diagonal of matrix
static double cart_gain_xd_scale[N_CART+1]; // diagonal of matrix
static double cart_gain_a_scale[N_CART+1];  // diagonal of matrix
static double cart_gain_ad_scale[N_CART+1]; // diagonal of matrix
static double cart_gain_integral;

static double trans_mult;
static double trans_period = 1.0;
static double speed_mult = 1.0;

// global functions 
extern "C" void
add_cilantro_track_task( void );

// local functions
static int  init_cilantro_track_task(void);
static int  run_cilantro_track_task(void);
static int  change_cilantro_track_task(void);

// external functions
extern "C" void
init_sm_controllers(void);
extern "C" int
cartesianImpedanceSimpleJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double *gain_x_scale,
			   double *gain_xd_scale,
			   double *gain_a_scale,
			   double *gain_ad_scale);
extern "C" int
cartesianImpedanceModelJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double *gain_x_scale,
			   double *gain_xd_scale,
			   double *gain_a_scale,
			   double *gain_ad_scale);


/*****************************************************************************
******************************************************************************
Function Name	: add_cilantro_track_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_cilantro_track_task( void )
{
  int i, j;
  
  addTask("Cilantro Tracking Task", init_cilantro_track_task, 
	  run_cilantro_track_task, change_cilantro_track_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_cilantro_track_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_cilantro_track_task(void)
{
  int j, i;
  int ans;
  static int firsttime = TRUE;
  
  if (firsttime){
    firsttime = FALSE;
    
    freq = 0.1; // frequency
    amp  = 0.1; // amplitude

    for (i=1; i<=N_CART; ++i) {
      cart_gain_x_scale[i]  = 1;
      cart_gain_xd_scale[i] = 1;
      cart_gain_a_scale[i]  = 1; 
      cart_gain_ad_scale[i] = 1;
    }
    cart_gain_integral = 0.0;

    init_sm_controllers();
    
  }

  // here we need the initialization of UDP communication with Cilantro tracker

  // prepare going to the default posture
  bzero((char *)&(target[1]),N_DOFS*sizeof(target[1]));
  for (i=1; i<=N_DOFS; i++)
    target[i] = joint_default_state[i];

  // go to the target using inverse dynamics (ID)
  if (!go_target_wait_ID(target)) 
    return FALSE;

  // initialize the cartesian control
  bzero((char *)&cdes,sizeof(cdes));
  bzero((char *)&cdes_orient,sizeof(cdes_orient));
  
  // speed change?
  get_double("Speed multiplier?",speed_mult,&speed_mult);
  if (speed_mult <= 0 || speed_mult > 2)
    speed_mult = 1.0;


  for (j=1; j<=N_CART; ++j) {
    
    // by default, we assume only positioon and no orientation control
    stats[j] = TRUE;
    stats[j + N_CART] = FALSE;
    
    // set cartesian target and current desired to current state to be safe
    cdes[HAND].x[j] = cart_des_state[HAND].x[j];
  }
  
  for (j=1; j<=N_QUAT; ++j) {
    cdes_orient[HAND].q[j] = cart_des_orient[HAND].q[j];
  }

  // the current cartesian state is used as base state for a simple cartesian sine movement
  cart_base_state = cart_des_state[HAND];

  // ready to go
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  // only go when user really types the right thing
  if (ans != 1) 
    return FALSE;

  start_time = task_servo_time;
  printf("start time = %.3f, task_servo_time = %.3f\n", 
	 start_time, task_servo_time);

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_cilantro_track_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_cilantro_track_task(void)
{
  int j, i;

  double task_time;
  double omega;
  int    dof;

  // NOTE: all array indices start with 1 in SL

  task_time = task_servo_time - start_time;
  omega     = 2.0*PI*freq*speed_mult;

  // smooth out onset of sine motion to avoid discontinuous signals
  trans_mult = task_time/trans_period;
  if (trans_mult > 1.0)
    trans_mult = 1.0;

  // sine motion in cartesian 3D space
  for (i=1; i<=N_CART; ++i) {
    cdes[HAND].x[i]    = cart_base_state.x[i] + amp*trans_mult*sin(i*omega*task_time);
    cdes[HAND].xd[i]   = trans_mult*amp*omega*cos(i*omega*task_time);
    cdes[HAND].xdd[i]  =-trans_mult*amp*omega*omega*sin(i*omega*task_time);
  }

  // this is passed on to the impedance controller
  cartesianImpedanceSimpleJt(cdes, cdes_orient, joint_des_state, joint_opt_state, stats,
			     cart_gain_integral,
			     cart_gain_x_scale,
			     cart_gain_xd_scale,
			     cart_gain_a_scale,
			     cart_gain_ad_scale);

  if (task_time == 1)
    sendCommandLineCmd("saveData");
  

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_cilantro_track_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_cilantro_track_task(void)
{
  int    ivar = 0;
  double dvar = 0;

  get_int("This is how to enter an integer variable",ivar,&ivar);
  get_double("This is how to enter a double variable",dvar,&dvar);

  return TRUE;

}

