/*============================================================================
==============================================================================
                      
                              impedance_test_task.c
 
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

/* defines */
#define  START_GAIN 0.0 // START_GAIN 0.0
enum Controllers {
  CART_IMPEDANCE_SIMPLE_JT,
  INVKIN_VEL_COMP_TORQUE,
  INVKIN_VEL_INV_DYN,
  HSU,
  KHATIB,
  DYN_DECOUP_NO_M,
  DYN_DECOUP_M,
  ACC_FORCE,
  ACC_FORCE_NO_M,
  N_CONTROLLERS
};

static char controller_names[][30] = {
  {"Cart.Impedance with simple J'"},
  {"Vel.Inv.Kin+Computed Torque"},
  {"Vel.Inv.Kin+Inv.Dyn"},
  {"Hsu"},
  {"Khatib"},
  {"Dyn.Decoup.+No M Premult"},
  {"Dyn.Decoup.+ M Premult"},
  {"Acc.+Force+M (Simple Hsu)"},
  {"Acc.+Force no M (Simple Hsu)"}
};

static int INVKIN_MODE = CART_IMPEDANCE_SIMPLE_JT;

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
static double     cart[N_ENDEFFS*6+1];
static SL_Cstate  ctarget[N_ENDEFFS+1];
static SL_quat    ctarget_quat[N_ENDEFFS+1];    /* Cartesian orientation target (quaternion) *new */
static My_Crot    ctarget_rot[N_ENDEFFS+1];     /* Cartesian orientation target (rot-axis)   *new */
static double     corient_error[N_ENDEFFS*3+1]; /* Orientation error term *new */
static int        stats[N_ENDEFFS*6+1];
static SL_DJstate target[N_DOFS+1];
static SL_DJstate last_target[N_DOFS+1];
static int        freezeDOF[N_DOFS+1];
static int        firsttime = TRUE;
static int        last_frame_counter;
static double     setpoint[N_ENDEFFS+1][N_CART+1];
static double     max_dist=0.2;
static double     start_time     = 0;
static double     gain           = 10;
static double     gain_orient    = 10;       /* *new */
static double     gain_trans     = START_GAIN;
static double     ball_speed     = 0.5;
static double     ball_amp       = 1.0;       /* *new */
static double     rot_amp        = 0.0;       /* orientation amplitude *new */
static double     rot_speed      = 0.0;       /* orientation speed *new */
static int        count_no_frame = 0;
static int        use_invdyn     = TRUE;
static int        wiggle         = FALSE;
static int        use_orient     = TRUE;
static int        wait_ticks;
static double     ralpha = 0.05; // ralpha for HSU
// static double     ralpha = 0.005; 
static double     ralpha_acc = 1.0;
static double     ralpha_dyn = 3.0;
static double     kn = 2;
#define N_AUX 50
static double     vaux[N_AUX+1];
static int        scd_wait_counter;

static SL_Cstate  ball_state;

static double     force = 0;

static double controller_gain_th[N_DOFS+1];
static double controller_gain_thd[N_DOFS+1];
static double controller_gain_int[N_DOFS+1];

/* global functions */
void add_impedance_test_task(void);


/* local functions */
static int
inverseKinematicsAcc(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		     Vector cart, iVector status, double dt, double t);
static int
inverseKinematicsAccM(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		      Vector cart, iVector status, double dt, double t);
static int
inverseKinematicsHsu(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		     Vector cart, iVector status, double dt, double t);
static int
inverseKinematicsKhatib(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			Vector cart, iVector status, double dt, double t);
static int
inverseKinematicsDynDecoupM(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			Vector cart, iVector status, double dt, double t);
static int
inverseKinematicsDynDecoupNoM(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			Vector cart, iVector status, double dt, double t);

static int
cartesianImpedanceSimpleJt(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			   Vector cart, iVector status, double dt, double t);


static int    init_impedance_test_task(void);
static int    run_impedance_test_task(void);
static int    change_impedance_test_task(void);
static void   init_vars(void);
static int    simulate_ball(void);
static int    init_filters(void);
static double filt(double raw, Filter *fptr);
static int    compute_target_orient(void); /* moving target orientation */
static void   quatDerivativesToAngVelAcc(SL_quat *q);
static Matrix SL_inertiaMatrix(SL_Jstate *lstate, SL_Cstate *cbase, 
			       SL_quat *obase, SL_endeff *leff);


/*****************************************************************************
******************************************************************************
Function Name	: add_impedance_test_task
Date		: Feb 2019
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_impedance_test_task( void )
{
  int i, j;
  char string[100];
  
  addTask("Impedance Test Task", init_impedance_test_task, 
	  run_impedance_test_task, change_impedance_test_task);

  // for debugging 
  for (i=1; i<=N_DOFS; ++i)
    freezeDOF[i] = FALSE;

  /* read the control gains  */
  if (!read_gains(config_files[GAINS],controller_gain_th, 
		  controller_gain_thd, controller_gain_int))
    return;


}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_impedance_test_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_impedance_test_task(void)
{
  int    j, i;
  char   string[100];
  int    ans;
  int    flag = FALSE;
  static int firsttime = TRUE;

  double aux_r_norm; /* tmp variable to check the norm of the rotation axis *new */

  if (firsttime) {

    /* initialize the filters */
    firsttime = FALSE;
    init_filters();
    for (i=1; i<=N_DOFS; ++i) 
      fthdd[i].cutoff = 5;

    /* add variables to data collection */
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
      addVarToCollect((char *)&(ctarget_quat[i].q[_Q0_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q1",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].q[_Q1_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q2",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].q[_Q2_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q3",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].q[_Q3_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_q0d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qd[_Q0_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q1d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qd[_Q1_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q2d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qd[_Q2_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q3d",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qd[_Q3_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_q0dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qdd[_Q0_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q1dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qdd[_Q1_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q2dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qdd[_Q2_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_q3dd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].qdd[_Q3_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_ad",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].ad[_A_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_bd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].ad[_B_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_gd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].ad[_G_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_ct_add",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].add[_A_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_bdd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].add[_B_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_ct_gdd",cart_names[i]);
      addVarToCollect((char *)&(ctarget_quat[i].add[_G_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_corient_e1",cart_names[i]);
      addVarToCollect((char *)&(corient_error[(i-1)*3 + _A_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_corient_e2",cart_names[i]);
      addVarToCollect((char *)&(corient_error[(i-1)*3 + _B_]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_corient_e3",cart_names[i]);
      addVarToCollect((char *)&(corient_error[(i-1)*3 + _G_]),string,"-", DOUBLE,FALSE);

      sprintf(string,"%s_cart_ref_x",cart_names[i]);
      addVarToCollect((char *)&(cart[(i-1)*6 + 1]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_y",cart_names[i]);
      addVarToCollect((char *)&(cart[(i-1)*6 + 2]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_z",cart_names[i]);
      addVarToCollect((char *)&(cart[(i-1)*6 + 3]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_a",cart_names[i]);
      addVarToCollect((char *)&(cart[(i-1)*6 + 4]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_b",cart_names[i]);
      addVarToCollect((char *)&(cart[(i-1)*6 + 5]),string,"-", DOUBLE,FALSE);
      sprintf(string,"%s_cart_ref_g",cart_names[i]);
      addVarToCollect((char *)&(cart[(i-1)*6 + 6]),string,"-", DOUBLE,FALSE);
      
      
    }
    
    for (i=1; i<=N_AUX; ++i) {
      sprintf(string,"vaux_%d",i);
      addVarToCollect((char *)&(vaux[i]),string,"-", DOUBLE,FALSE);
    }

    updateDataCollectScript();


  } else {

    /* zero the filters */
    for (i=1; i<=N_DOFS; ++i) 
      for (j=0; j<=FILTER_ORDER; ++j)
	fthdd[i].raw[j] = fthdd[i].filt[j] = 0;

  }

  /* check whether any other task is running */
  if (strcmp(current_task_name,NO_TASK) != 0) {
    printf("Task can only be started if no other task is running!\n");
    return FALSE;
  }

   /* initialize some variables */
  init_vars();
  time_step = 1./(double)task_servo_rate;

  /* which controller */
  printf("Choose controller:\n");
  for (i=0; i<N_CONTROLLERS; ++i) {
    printf("     %30s : %2d\n",controller_names[i],i);
  }
  get_int("Which Controller?",INVKIN_MODE,&INVKIN_MODE);

  get_int("Use Orientation?",use_orient,&use_orient);
  if (use_orient!=1 && use_orient !=0) 
    use_orient = 0;


  if (INVKIN_MODE==INVKIN_VEL_COMP_TORQUE || INVKIN_MODE==INVKIN_VEL_INV_DYN){
  /* input the movement gain */
    do{
      get_double("Tracking Gain? (0-20)",gain,&gain);
    } while (gain > 20 || gain < 0);

    if (use_orient) 
      do{
	get_double("Orientation Gain? (0-100)",gain_orient,&gain_orient);
      } while  (gain_orient > 100 || gain_orient < 0);
  }
  else{

    if (INVKIN_MODE == CART_IMPEDANCE_SIMPLE_JT) { // some reasonable gains
      static int firsttime = TRUE;
      if (firsttime) {
	firsttime = FALSE;
	gain = 250;
	gain_orient = 40;
      }
    }
    
    do{
      get_double("Tracking Gain? (0-1000)",gain,&gain);
    } while (gain > 1000 || gain < 0);

    if (use_orient) 
      do{
	get_double("Orientation Gain? (0-1000)",gain_orient,&gain_orient);
      } while  (gain_orient > 1000 || gain_orient < 0);
  }

  /* ball speed */
  do{
    get_double("Simulated ball speed (0.0-5.0)",ball_speed,&ball_speed);
  } while (ball_speed < 0 || ball_speed > 5);
  
  do{
    get_double("Simulated ball amplitude (0.0-2.0)",ball_amp,&ball_amp);
  } while (ball_amp < 0 || ball_amp > 2);
  
  do{
    get_double("Tagret rotation speed (0.0-5.0)",rot_speed,&rot_speed);
  } while (rot_speed < 0 || rot_speed > 5);
  
  do{
    get_double("Target rotation amplitude (0.0-2.0)",rot_amp,&rot_amp);
  } while (rot_amp < 0 || rot_amp > 2);

  /* simulate the ball once to initialize variables */
  start_time = task_servo_time;
  simulate_ball();

  /* some randomness around the figure 8 ?*/
  get_int("Drifting?",wiggle,&wiggle);
  if (wiggle!=1 && wiggle !=0) 
    wiggle = 0;

    
  /* go to a save posture */
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

  /* initialize the cartesian movement */
  bzero((char *)&stats,sizeof(stats));
  for (i=1; i<=N_ENDEFFS; ++i) {
    stats[(i-1)*6+ _X_] = 1;
    stats[(i-1)*6+ _Y_] = 1;
    stats[(i-1)*6+ _Z_] = 1;
    setpoint[i][_X_] = ctarget[i].x[_X_] = 
      cart_des_state[i].x[_X_];
    setpoint[i][_Y_] = ctarget[i].x[_Y_] = 
      cart_des_state[i].x[_Y_];
    setpoint[i][_Z_] = ctarget[i].x[_Z_] = 
      cart_des_state[i].x[_Z_];
  }
  
  /* initialize the cartesian orientation movement */
  for (i=1; i<=N_ENDEFFS; ++i) {
    stats[(i-1)*6+ _A_ + N_CART] = use_orient; /* index 4 */
    stats[(i-1)*6+ _B_ + N_CART] = use_orient; /* index 5 */
    stats[(i-1)*6+ _G_ + N_CART] = use_orient; /* index 6 */
  }

  /* compute the desired target orientation */
  compute_target_orient();

  /* initialize the ball simulator */
  ball_state.x[_X_] = setpoint[1][_X_];
  ball_state.x[_Y_] = setpoint[1][_Y_];
  ball_state.x[_Z_] = setpoint[1][_Z_];
  
  /* ready to go */
  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  if (ans != 1) 
    return FALSE;
  
  last_frame_counter = frame_counter;
  gain_trans = START_GAIN;
  count_no_frame = 0;
  wait_ticks = 1;
  scd_wait_counter = 10*task_servo_rate; /* for scd */

  return TRUE;

}

static void
init_vars(void) 
{
  if (firsttime) {
    firsttime = FALSE;
    bzero((char *)&stats,(N_ENDEFFS*6+1)*sizeof(int));
    bzero((char *)&cart,(N_ENDEFFS*6+1)*sizeof(double));
    bzero((char *)&(ctarget[1]),N_ENDEFFS*sizeof(ctarget[1]));
  }
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_impedance_test_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_impedance_test_task(void)
{
  int j, i;
  double sum=0;
  double dist;
  double aux;

  /* simulate the ball if needed */
  simulate_ball();
  compute_target_orient(); /* compute target orientation */

  if (--wait_ticks > 0) {
    return TRUE;
  } else if (wait_ticks == 0) {
    start_time = task_servo_time;
  }

  if (scd_wait_counter==0){
    scd();
  }
  scd_wait_counter--;

  // increase the gain gradually 
  gain_trans += 0.001*(1-gain_trans);

  for (i=1; i<=N_ENDEFFS; ++i) {

    // check distance to target
    dist = sqrt(sqr(ball_state.x[_X_] - setpoint[i][_X_]) +
		sqr(ball_state.x[_Y_] - setpoint[i][_Y_]) +
		sqr(ball_state.x[_Z_] - setpoint[i][_Z_]));
    
    if ( dist < max_dist) {

      for (j= _X_; j<= _Z_; ++j) {	
	ctarget[i].xdd[j] = ball_state.xdd[j];
	ctarget[i].xd[j]  = ball_state.xd[j];
	ctarget[i].x[j]   = ball_state.x[j];
      }

    } else {

      // project the target onto the workspace sphere
      for (j= _X_; j<= _Z_; ++j) {
	ctarget[i].xdd[j] = ball_state.xdd[j];
	ctarget[i].xd[j]  = ball_state.xd[j];
	ctarget[i].x[j] = setpoint[i][j] + 
	  (ball_state.x[j] - setpoint[i][j])*max_dist/dist;
      }	

      // inner product of normalized (x-setpoint) and xd gives the velocity
      // component that is perpendicular to the workspace sphere
      aux = 0.0;
      for (j= _X_; j<= _Z_; ++j) {
	aux += (ctarget[i].x[j] - setpoint[i][j])/max_dist *  ctarget[i].xd[j];
      }
      // subtract the perpendicular component from velocity with a 
      // smoothing factor
      for (j= _X_; j<= _Z_; ++j) {
	ctarget[i].xd[j] -= (ctarget[i].x[j] - setpoint[i][j])/
	  max_dist * aux * (1.-max_dist/dist);
      }
      // the same approache for accelerations
      aux = 0.0;
      for (j= _X_; j<= _Z_; ++j) {
	aux += (ctarget[i].x[j] - setpoint[i][j])/max_dist *  ctarget[i].xdd[j];
      }
      for (j= _X_; j<= _Z_; ++j) {
	ctarget[i].xdd[j] -= (ctarget[i].x[j] - setpoint[i][j])/
	  max_dist * aux * (1.-max_dist/dist);
      }
    }

  } 

  // the inverse kinematics controller needs the current state as input;
  // the is no notion of a desire joint state, i.e., it is equal to the
  // current state as only the operational space servo is active
  for (i=1; i<=N_DOFS; ++i) {
    target[i].th   = joint_state[i].th;
    target[i].thd  = joint_state[i].thd;
    target[i].thdd = 0.0;
  }

  /* compute orientation error term for quaterion feedback control *new */
  quatErrorVector(ctarget_quat[1].q,cart_orient[1].q,corient_error);
  
  switch (INVKIN_MODE) {

  case CART_IMPEDANCE_SIMPLE_JT:

    // prepare the impdance controller, i.e., compute operational
    // space force command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    ((ctarget[i].xd[j]  - cart_state[i].xd[j]) * 2.*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    ((ctarget_quat[i].ad[j] - cart_orient[i].ad[j]) *0.025 * 2.0 * sqrt(gain_orient) - 
	    corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    
    // this computes the joint space torque
    if (!cartesianImpedanceSimpleJt(target,endeff,joint_opt_state,
				    cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }

    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = 0.0;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    // add feedforward torques from impedance controller
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].uff   += target[i].uff;
    }

    
    break;

  case DYN_DECOUP_M:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xdd[j] + 
	     (ctarget[i].xd[j]  - cart_state[i].xd[j]) * 0.5 * 2.*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].add[j] + 
	     (ctarget_quat[i].ad[j] - cart_orient[i].ad[j])* 2.0 * sqrt(gain_orient) + 
	     - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    
    // this computes the target.thdd commands
    if (!inverseKinematicsDynDecoupM(target,endeff,joint_opt_state,
				     cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }
    
    
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = target[i].thdd;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    break;


  case ACC_FORCE:
    

    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xdd[j] + 
	     (ctarget[i].xd[j]  - cart_state[i].xd[j]) * 0.5 * 2.*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].add[j] + 
	     (ctarget_quat[i].ad[j] - cart_orient[i].ad[j])* 2.0 * sqrt(gain_orient) + 
	     - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    
    // this computes the target.thdd commands
    if (!inverseKinematicsAccM(target,endeff,joint_opt_state,
			      cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }
    
    
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = target[i].thdd;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    break;


  case ACC_FORCE_NO_M:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xdd[j] + 
	     (ctarget[i].xd[j]  - cart_state[i].xd[j]) * 0.5 * 2.*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].add[j] + 
	     (ctarget_quat[i].ad[j] - cart_orient[i].ad[j])* 2.0 * sqrt(gain_orient) + 
	     - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    

    // this computes the target.thdd commands
    if (!inverseKinematicsAcc(target,endeff,joint_opt_state,
			      cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }
    
    
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = target[i].thdd;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].uff   += target[i].uff;
    }
    
    break;


  case DYN_DECOUP_NO_M:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xdd[j] + 
	     (ctarget[i].xd[j]  - cart_state[i].xd[j]) * 2.0 * 2.0*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].add[j] + 
	     (ctarget_quat[i].ad[j] - cart_orient[i].ad[j])* 2.0 * sqrt(gain_orient) +
	     - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    
    // this computes the target.thdd commands
    if (!inverseKinematicsDynDecoupNoM(target,endeff,joint_opt_state,
				       cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }
    
    
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = target[i].thdd;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    // need to add the null space term, store in target.uff
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].uff   += target[i].uff;
    }
    
    break;


  case KHATIB:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xdd[j] + 
	     (ctarget[i].xd[j]  - cart_state[i].xd[j]) * 0.5 * 2.*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].add[j] + 
	     (ctarget_quat[i].ad[j] - cart_orient[i].ad[j])* 2.0 * sqrt(gain_orient) + 
	     - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    
    // this computes the target.thdd commands
    if (!inverseKinematicsKhatib(target,endeff,joint_opt_state,
				 cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }
    
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = target[i].thdd;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = target[i].uff;
    }
    
    break;


  case HSU:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xdd[j] + 
	     (ctarget[i].xd[j]  - cart_state[i].xd[j]) * 0.5 * 2.*sqrt(gain) +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].add[j] + 
	     (ctarget_quat[i].ad[j] - cart_orient[i].ad[j])* 2.0 * sqrt(gain_orient) + 
	     - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }
    
    // this computes the target.thdd commands
    if (!inverseKinematicsHsu(target,endeff,joint_opt_state,
			      cart,stats,time_step,task_servo_time)) {
      freeze();
      return FALSE;
    }
    
    
    for (i=1; i<=N_DOFS; ++i) {
      joint_des_state[i].thdd  = target[i].thdd;
      joint_des_state[i].thd   = joint_state[i].thd;
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);

    break;

  case INVKIN_VEL_COMP_TORQUE:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xd[j] + 
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].ad[j] + 
	   - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }

    if (!inverseKinematics(target,endeff,joint_opt_state,
			   cart,stats,time_step)) {
      freeze();
      return FALSE;
    }

    for (i=1; i<=N_DOFS; ++i) {
      
      /*
	joint_des_state[i].thdd += 0.25*
	((target[i].thd-joint_des_state[i].thd)*
	(double)task_servo_rate-joint_des_state[i].thdd);
      */
      aux = (target[i].thd-joint_des_state[i].thd)*(double)task_servo_rate;
      joint_des_state[i].thdd  = filt(aux,&(fthdd[i]));
      joint_des_state[i].thd   = target[i].thd;
      joint_des_state[i].th    = target[i].th;
      joint_des_state[i].uff   = 0.0;
      
    }
    
    if (use_invdyn) 
      SL_InvDyn(NULL,joint_des_state,endeff,&base_state,&base_orient);
    
    // well, without inverse dynamics, this controller doesnot work
    /*
    if (!use_idservo && use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);
    */

    break;

  case INVKIN_VEL_INV_DYN:
    
    // prepare the inverse kinematic controller, i.e., compute operational
    // space acceleration command 
    for (i=1; i<=N_ENDEFFS; ++i) {
      for (j= _X_; j<= _Z_; ++j) {
	if (stats[(i-1)*6+j]) {
	  cart[(i-1)*6+j] = 
	    (ctarget[i].xd[j] +
	     (ctarget[i].x[j]  - cart_state[i].x[j]) * gain) * gain_trans;
	}
      }

      for (j= _A_; j<= _G_ ; ++j) { /* orientation */
	if (stats[(i-1)*6+ N_CART + j]) {
	  cart[(i-1)*6 + N_CART + j] = 
	    (ctarget_quat[i].ad[j] + 
	   - corient_error[(i-1)*3+j] * gain_orient) * gain_trans; 
	}
      }

    }

    if (!inverseKinematics(target,endeff,joint_opt_state,
			 cart,stats,time_step)) {
      freeze();
      return FALSE;
    }

    for (i=1; i<=N_DOFS; ++i) {
      // get acc from numerical differentiation
      aux = (target[i].thd-last_target[i].thd)*(double)task_servo_rate;
      joint_des_state[i].thdd  = filt(aux,&(fthdd[i]));
      // need damping in joint space
      joint_des_state[i].thd   = target[i].thd;
      // otherwise use inv. dynamics control properly
      joint_des_state[i].th    = joint_state[i].th;
      joint_des_state[i].uff   = 0.0;
      // keep track of previous time step target
      last_target[i].th   = target[i].th;
      last_target[i].thd  = target[i].thd;
      last_target[i].thdd = target[i].thdd;
    }
    
    // well, without inverse dynamics, this controller doesnot work
    if (use_invdyn) 
      SL_InvDyn(joint_state,joint_des_state,endeff,&base_state,&base_orient);


    break;

  default:

    break;

  }


  // focus only on selected DOFs
  for (i=1; i<=N_DOFS; ++i) {
    if (freezeDOF[i]) {
      joint_des_state[i].th   = joint_default_state[i].th;
      joint_des_state[i].thd  = 0.0;
      joint_des_state[i].thdd = 0.0;
      joint_des_state[i].uff  = 0.0;
    }
  }



  uext_sim[4].f[1] = force;
  sendUextSim();

  return TRUE;
  
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_impedance_test_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_impedance_test_task(void)
{
  int j, i;
  char string[100];
  double aux;
  double aux_r[N_CART+1];
  double aux_r_norm;

  get_double("Max dist",max_dist,&max_dist);
  for (i=1; i<=N_ENDEFFS; ++i) {
    get_double("Setpoint X",setpoint[i][_X_],&setpoint[i][_X_]);
    get_double("Setpoint Y",setpoint[i][_Y_],&setpoint[i][_Y_]);
    get_double("Setpoint Z",setpoint[i][_Z_],&setpoint[i][_Z_]);

    do{
    get_double("Target Cart rot X axis",ctarget_rot[i].r[_A_], &aux_r[_A_]);
    get_double("Target Cart rot Y axis",ctarget_rot[i].r[_B_], &aux_r[_B_]);
    get_double("Target Cart rot Z axis",ctarget_rot[i].r[_G_], &aux_r[_G_]);

    aux_r_norm = sqrt(aux_r[_A_]*aux_r[_A_] + aux_r[_B_]*aux_r[_B_]+ aux_r[_G_]*aux_r[_G_]);
    } while (aux_r_norm < 0.001);

    ctarget_rot[i].r[_A_] = aux_r[_A_]/aux_r_norm;
    ctarget_rot[i].r[_B_] = aux_r[_B_]/aux_r_norm;
    ctarget_rot[i].r[_G_] = aux_r[_G_]/aux_r_norm;

    do{
      get_double("Target Cart rot angle phi (-180~180 deg)",ctarget_rot[i].phi*180.0/PI, &aux);
    } while (aux < -180 || aux > 180);
    
    ctarget_rot[i].phi  = aux * PI/180.0;

    ctarget_quat[i].q[_Q0_] = cos(ctarget_rot[i].phi/2.0);
    ctarget_quat[i].q[_Q1_] = ctarget_rot[i].r[_A_]*sin(ctarget_rot[i].phi/2.0);
    ctarget_quat[i].q[_Q2_] = ctarget_rot[i].r[_B_]*sin(ctarget_rot[i].phi/2.0);
    ctarget_quat[i].q[_Q3_] = ctarget_rot[i].r[_G_]*sin(ctarget_rot[i].phi/2.0);

    printf("Target cart quaternion = [%.3f, %.3f, %.3f, %.3f]\n",
	   ctarget_quat[i].q[_Q0_],
	   ctarget_quat[i].q[_Q1_],
	   ctarget_quat[i].q[_Q2_],
	   ctarget_quat[i].q[_Q3_]);

  }

  //  ball simulation parameters
  do{
    get_double("Simulated ball speed (0.0-5.0)",ball_speed,&ball_speed);
  } while (ball_speed < 0 || ball_speed > 5);
  
  do{
    get_double("Simulated ball amplitude (0.0-2.0)",ball_amp,&ball_amp);
  } while (ball_amp < 0 || ball_amp > 2.0);
  
  do{
    get_double("Taret rotation speed (0.0-5.0)",rot_speed,&rot_speed);
  } while (rot_speed < 0 || rot_speed > 5);
  
  do{
    get_double("Target rotation amplitude (0.0-2.0)",rot_amp,&rot_amp);
  } while (rot_amp < 0 || rot_amp > 2);

  //gain_trans = START_GAIN;

  /* free DOFs */
  /*
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    sprintf(string,"Freeze %s:",joint_names[i]);
    get_int(string,freezeDOF[i],&j);
    if (j==0)
      freezeDOF[i] = FALSE;
    else
      freezeDOF[i] = TRUE;
  }
  */

  get_double("ralpha for HSU",ralpha,&aux);
  if (aux < 0 || aux > 10)
    ; // don't change
  else
    ralpha = aux;

  get_double("ralpha for AccForce",ralpha_acc,&aux);
  if (aux < 0 || aux > 10)
    ; // don't change
  else
    ralpha_acc = aux;

  get_double("ralpha for DynDecoup",ralpha_dyn,&aux);
  if (aux < 0 || aux > 10)
    ; // don't change
  else
    ralpha_dyn = aux;

  get_double("kn for HSU",kn,&aux);
  if (aux < 0 || aux > 25)
    ; // don't change
  else
    kn = aux;

  get_double("perturbation force",force,&force);

  return TRUE;

}

/*****************************************************************************
******************************************************************************
  Function Name	: simulate_ball
  Date		: Dec. 1997

  Remarks:

  calculates the state of a simulated ball and copies it into the vision
  states

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
simulate_ball(void)
{
  int j, i;
  double w1,w2,w3,w4;
  double dt;

  w1 = 2.*PI*0.25*ball_speed;
  w2 = 2.*PI*0.113;
  w3 = 2.*PI*0.078633;
  w4 = 2.*PI*0.145633;
  dt = task_servo_time-start_time;

  ball_state.x[_Y_] = ball_amp*0.15*sin(w1*dt) + 
    setpoint[1][_Y_] + ball_amp*wiggle * 0.05*sin(w2*dt);
  ball_state.x[_X_] = setpoint[1][_X_] + ball_amp*wiggle * 0.05 * sin(w3*dt);
  ball_state.x[_Z_] = ball_amp*0.1*sin(2.*w1*dt) + setpoint[1][_Z_] + ball_amp*wiggle * 0.03*sin(w4*dt);

  ball_state.xd[_Y_] = ball_amp*0.15*w1*cos(w1*dt) + ball_amp*wiggle * 0.05*w2*cos(w2*dt);
  ball_state.xd[_X_] = ball_amp*wiggle * 0.05 * w3*cos(w3*dt);
  ball_state.xd[_Z_] = ball_amp*0.1*2*w1*cos(2.*w1*dt) + ball_amp*wiggle * 0.03*w4*cos(w4*dt);

  ball_state.xdd[_Y_] = -0.15*ball_amp*sqr(w1)*sin(w1*dt) - ball_amp*wiggle * 0.05*sqr(w2)*sin(w2*dt);
  ball_state.xdd[_X_] = -wiggle * ball_amp*0.05 * sqr(w3)*sin(w3*dt);
  ball_state.xdd[_Z_] = -0.1*ball_amp*sqr(2*w1)*sin(2.*w1*dt) - ball_amp*wiggle * 0.03*sqr(w4)*sin(w4*dt);
  
  // send ball at lower frequency
  if (task_servo_calls%8==0) {
    float pos[N_CART+1];
 
    pos[_X_] =  ball_state.x[_X_];
    pos[_Y_] =  ball_state.x[_Y_];
    pos[_Z_] =  ball_state.x[_Z_];

    sendUserGraphics("ball",&(pos[_X_]), N_CART*sizeof(float));
  }

  return TRUE;

}

/*****************************************************************************
******************************************************************************
Function Name	: inverseKinematicsHsu
Date		: June 1999
   
Remarks:

        computes the resolved motion acceleration based solution to 
        the inverse kinematics, i.e., a desired acceleration in
        operational space is transformed in a desired acceleration
        in joint space (such that an inverse dynamics controller
        can be employed). 

        The algorithm is based on:
        Hsu P, Hauser J, Sastry S (1989) Dynamic control of redundant 
        manipulators. Journal of Robotic Systems 6: 133-148

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

 the function updates the state by adding the appropriate joint accelerations

*****************************************************************************/
static int
inverseKinematicsHsu(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		     Vector cart, iVector status, double dt, double t)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  P, O, T;
  static Matrix  B, last_B, dBdt;
  static iVector ind;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static double  last_t;       
  static Vector  e, en;
  static Vector  Je;
  //  double         ridge = 1.e-6;
  double         ridge = 1.e-5;
  double w[N_DOFS+1] = {0.0,
			250.0,
			250.0,
			250.0,
			250.0,
			250.0,
			500.0,
			250.0};

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    last_B = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    dBdt   = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    T      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    dJdtthd = my_vector(1,6*N_ENDEFFS);
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    Je     = my_vector(1,6*N_ENDEFFS);
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

  /* compute the time derivative of the Jacobian and the pseudo-inverse 
     from numerical differentiation */

  /* first check whether last_B and last_J are valid */
  if (  last_t + dt != t) {
    /* this will set the time derivatives to zero at this step */
    mat_equal(B,last_B);
  }
  mat_sub(B,last_B,dBdt);
  mat_mult_scalar(dBdt,1./dt,dBdt);

  //hack
  //mat_zero(dBdt);
  //mat_zero(dJdt);

  /* need dJdt * thd */
  for (i=1; i<=count; ++i) {
    dJdtthd[ind[i]] = 0.0;
    for (j=1; j<=N_DOFS; ++j)
      dJdtthd[ind[i]] += dJdt[ind[i]][j]*state[j].thd;
  }

  /* this provides the RANGE part of the solution to the joint space acceleration */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thdd = 0;
    for (j=1; j<=count; ++j) {
      state[i].thdd += B[i][j] * (cart[ind[j]] - dJdtthd[ind[j]]);
    }
  }

  /* next comes the optimization part */

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

  /* compute the joint velocity error and its NULL space projection */
  for (i=1; i<=N_DOFS; ++i) {
    e[i] = w[i]*(rest[i].th - state[i].th) - state[i].thd;
  }
  mat_vec_mult(O,e,en);

  /* compute J*e */
  for (i=1; i<=count; ++i) {
    Je[i] = 0.0;
    for (j=1; j<=N_DOFS; ++j) {
      Je[i] += J[ind[i]][j] * e[j];  
    }
  }

  /* compute dJdt * B , and store in P */
  for (i=1; i<=count; ++i) {
    for (j=1; j<=count; ++j) {
      P[i][j] = 0.0;
      for (n=1; n<=N_DOFS; ++n) {
	P[i][j] += dJdt[ind[i]][n]*B[n][j];
      }
    }
  }

  /* compute B*P+dBdt, and store in T */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=count; ++j) {
      T[i][j] = dBdt[i][j];
      for (n=1; n<=count; ++n)
	T[i][j] += B[i][n]*P[n][j];
    }
  }

  /* now comes O*(Kn*en-thd*w)-T*Je */
  /* replace en by (Kn*en-thd*w) */
  for (i=1; i<=N_DOFS; ++i)
    en[i] = +kn*en[i] - state[i].thd*w[i];

  /* add the first optimization part to the accelerations */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=N_DOFS; ++j) {
      state[i].thdd += O[i][j] * en[j] * ralpha;
    }
  }

  /* add the second optimization part to the accelerations */
  for (i=1; i<=N_DOFS; ++i) 
    for (j=1; j<=count; ++j)
      state[i].thdd -= T[i][j]*Je[j]*ralpha;
  
  /* update the "last" variables */
  last_t = t;
  mat_equal(B,last_B);

  return TRUE;

}


/*****************************************************************************
******************************************************************************
Function Name	: inverseKinematicsAcc
Date		: June 1999
   
Remarks:

        computes the resolved motion acceleration based solution to 
        the inverse kinematics, i.e., a desired acceleration in
        operational space is transformed in a desired acceleration
        in joint space (such that an inverse dynamics controller
        can be employed). 

        The algorithm is a simplified version of Hsu, not running the
        null space through the M matrix
        
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

 the function updates the state by adding the appropriate joint accelerations

*****************************************************************************/
static int
inverseKinematicsAcc(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
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
  //  double         ridge = 1.e-6;
  double         ridge = 1.e-5;

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

  /* need dJdt * thd */
  for (i=1; i<=count; ++i) {
    dJdtthd[ind[i]] = 0.0;
    for (j=1; j<=N_DOFS; ++j)
      dJdtthd[ind[i]] += dJdt[ind[i]][j]*state[j].thd;
  }

  /* this provides the RANGE part of the solution to the joint space acceleration */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thdd = 0;
    for (j=1; j<=count; ++j) {
      state[i].thdd += B[i][j] * (cart[ind[j]] - dJdtthd[ind[j]]);
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
    double fac=0.5;
    e[i] = 
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd;
    vaux[i] = e[i];
  }
  mat_vec_mult(O,e,en);

  /* return this as a PD command in uff */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff = en[i];
    vaux[i+7] = en[i];
  }

  // data visualization only 
  if (1) {
    // compute inertia matrix and its inverse
    M = SL_inertiaMatrix(joint_state, &base_state, &base_orient, endeff);
    my_inv_ludcmp(M, N_DOFS-N_DOFS_EST_SKIP, invM);
    mat_vec_mult_size(invM,N_DOFS-N_DOFS_EST_SKIP,N_DOFS-N_DOFS_EST_SKIP,
		      en,N_DOFS-N_DOFS_EST_SKIP,
		      en);

    for (i=1; i<=count; ++i) {
      vaux[i+14] = 0.0;
      for (j=1; j<=N_DOFS; ++j)
	vaux[i+14] += J[ind[i]][j]*en[j];
    }

  }

  return TRUE;

}


/*****************************************************************************
******************************************************************************
Function Name	: inverseKinematicsAccM
Date		: June 1999
   
Remarks:

        computes the resolved motion acceleration based solution to 
        the inverse kinematics, i.e., a desired acceleration in
        operational space is transformed in a desired acceleration
        in joint space (such that an inverse dynamics controller
        can be employed). 

        The algorithm is a simplified version of Hsu, running the null
        space through the M matrix
        
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

 the function updates the state by adding the appropriate joint accelerations

*****************************************************************************/
static int
inverseKinematicsAccM(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		     Vector cart, iVector status, double dt, double t)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B;
  static iVector ind;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static double  last_t;       
  static Vector  e, en;
  static Vector  Je;
  //  double         ridge = 1.e-6;
  double         ridge = 1.e-5;
  double w[N_DOFS+1] = {0.0,
			250.0,
			50.0,
			50.0,
			50.0,
			100.0,
			250.0,
			100.0};

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

  /* need dJdt * thd */
  for (i=1; i<=count; ++i) {
    dJdtthd[ind[i]] = 0.0;
    for (j=1; j<=N_DOFS; ++j)
      dJdtthd[ind[i]] += dJdt[ind[i]][j]*state[j].thd;
  }

  /* this provides the RANGE part of the solution to the joint space acceleration */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thdd = 0;
    for (j=1; j<=count; ++j) {
      state[i].thdd += B[i][j] * (cart[ind[j]] - dJdtthd[ind[j]]);
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
    e[i] = 
      ralpha_acc*w[i]*(rest[i].th - state[i].th) - 
      2.*sqrt(ralpha_acc*w[i])/2.0 *state[i].thd;
  }
  mat_vec_mult(O,e,en);

  /* add the optimization part to the accelerations */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thdd += en[i];
  }

  return TRUE;

}


/*****************************************************************************
******************************************************************************
Function Name	: inverseKinematicsKhatib
Date		: June 1999
   
Remarks:

        operational space controller of Khatib
        
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

 the function updates the state by adding the appropriate joint accelerations

*****************************************************************************/
static int
inverseKinematicsKhatib(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			Vector cart, iVector status, double dt, double t)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B, M, invM, JinvM;
  static iVector ind;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static double  last_t;       
  static Vector  e, en;
  //double         ridge = 1.e-2;
  double         ridge = 1.e-5;

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    invM   = my_matrix(1,N_DOFS,1,N_DOFS);
    dJdtthd = my_vector(1,6*N_ENDEFFS);
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    JinvM  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
  }

  // compute inertia matrix and its inverse
  M = SL_inertiaMatrix(joint_state, &base_state, &base_orient, endeff);
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i)
    M[i][i] += ridge;
  my_inv_ludcmp(M, N_DOFS-N_DOFS_EST_SKIP, invM);

  /* how many contrained cartesian DOFs do we have? */
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  // compute J*inv(M) 
  for (i=1; i<=count; ++i) {
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      JinvM[ind[i]][j] = 0.0;
      for (n=1; n<=N_DOFS-N_DOFS_EST_SKIP; ++n) 
	JinvM[ind[i]][j] += J[ind[i]][n]*invM[n][j];
    }
  }

  /* build the inertia weighted pseudo-inverse according to the status
     information */
  mat_zero(P);
  for (i=1; i<=count; ++i) {
    for (j=1; j<=count; ++j) {
      for (n=1; n<=N_DOFS-N_DOFS_EST_SKIP; ++n) {
	P[i][j] += J[ind[i]][n] * JinvM[ind[j]][n];
      }
      if (i==j) 
	P[i][j] += ridge;
    }
  }

  /* invert the matrix */
  if (!my_inv_ludcmp(P, count, P)) {
    return FALSE;
  }

  /* build the B matrix, i.e., the inertia weighted pseudo-inverse */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    for (j=1; j<=count; ++j) {
      B[i][j]=0.0;
      for (n=1; n<=count; ++n) {
	B[i][j] += JinvM[ind[n]][i] * P[n][j];
      }
    }
  }

  /* compute C+g terms: "state" accelerations are already zeroed, while
   pos. and vel. are set to the current state */
  SL_InvDyn(NULL,state,endeff,&base_state,&base_orient);

  /* need dJdt * thd - JinvM*(C+g) */
  for (i=1; i<=count; ++i) {
    dJdtthd[ind[i]] = 0.0;
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j)
      dJdtthd[ind[i]] += dJdt[ind[i]][j]*state[j].thd - 
	JinvM[ind[i]][j] * state[j].uff;
  }

  /* this provides the RANGE part of the solution to the joint space acceleration */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    state[i].thdd = 0;
    for (j=1; j<=count; ++j) { 
      state[i].thdd += B[i][j] * (cart[ind[j]] - dJdtthd[ind[j]]);
    }
  }

  /**********************/
  /* the null space term */
  /**********************/

  /* compute the NULL space projection matrix; note that it is computationally
     inefficient to do this, since this is O(d^2), while all we really need is
     some matrix-vector multiplications, which are much cheaper */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      if (i==j) 
	O[i][j] = 1.0;
      else
	O[i][j] = 0.0;
      for (n=1; n<=count; ++n) {
	O[i][j] -= J[ind[n]][i] * B[j][n];
      }
    }
  }

  /* compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    double fac=0.5;
    e[i] = 
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd;
  }
  mat_vec_mult(O,e,en);

  /* return this as a PD command in uff */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    state[i].uff = en[i]; 
  }

  // add the M*state.thdd
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i)
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j)
      state[i].uff += M[i][j]*state[j].thdd;

  return TRUE;

}


/*****************************************************************************
******************************************************************************
Function Name	: inverseKinematicsDynDecoupM
Date		: June 1999
   
Remarks:

        operational space controller of Khatib
        
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

 the function updates the state by adding the appropriate joint accelerations

*****************************************************************************/
static int
inverseKinematicsDynDecoupM(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			    Vector cart, iVector status, double dt, double t)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B, M, invM, JinvM;
  static iVector ind;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static double  last_t;       
  static Vector  e, en;
  //  double         ridge = 1.e-2;
  double         ridge = 1.e-5;

  double w[N_DOFS+1] = {0.0,
			50.0,
			50.0,
			50.0,
			50.0,
			250.0,
			250.0,
			250.0};

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    invM   = my_matrix(1,N_DOFS,1,N_DOFS);
    dJdtthd = my_vector(1,6*N_ENDEFFS);
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    JinvM  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
  }

  // compute inertia matrix and its inverse
  M = SL_inertiaMatrix(joint_state, &base_state, &base_orient, endeff);
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i)
    M[i][i] += ridge;
  my_inv_ludcmp(M, N_DOFS-N_DOFS_EST_SKIP, invM);

  /* how many contrained cartesian DOFs do we have? */
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  // compute J*inv(M) 
  for (i=1; i<=count; ++i) {
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      JinvM[ind[i]][j] = 0.0;
      for (n=1; n<=N_DOFS-N_DOFS_EST_SKIP; ++n) 
	JinvM[ind[i]][j] += J[ind[i]][n]*invM[n][j];
    }
  }

  /* build the inertia weighted pseudo-inverse according to the status
     information */
  mat_zero(P);
  for (i=1; i<=count; ++i) {
    for (j=1; j<=count; ++j) {
      for (n=1; n<=N_DOFS-N_DOFS_EST_SKIP; ++n) {
	P[i][j] += J[ind[i]][n] * JinvM[ind[j]][n];
      }
      if (i==j) 
	P[i][j] += ridge;
    }
  }

  /* invert the matrix */
  if (!my_inv_ludcmp(P, count, P)) {
    return FALSE;
  }

  /* build the B matrix, i.e., the inertia weighted pseudo-inverse */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    for (j=1; j<=count; ++j) {
      B[i][j]=0.0;
      for (n=1; n<=count; ++n) {
	B[i][j] += JinvM[ind[n]][i] * P[n][j];
      }
    }
  }

  /* need dJdt * thd  */
  for (i=1; i<=count; ++i) {
    dJdtthd[ind[i]] = 0.0;
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j)
      dJdtthd[ind[i]] += dJdt[ind[i]][j]*state[j].thd; 
  }

  /* this provides the RANGE part of the solution to the joint space acceleration */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    state[i].thdd = 0;
    for (j=1; j<=count; ++j) { 
      state[i].thdd += B[i][j] * (cart[ind[j]] - dJdtthd[ind[j]]);
    }
  }

  /**********************/
  /* the null space term */
  /**********************/

  /* compute the NULL space projection matrix; note that it is computationally
     inefficient to do this, since this is O(d^2), while all we really need is
     some matrix-vector multiplications, which are much cheaper */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      if (i==j) 
	O[i][j] = 1.0;
      else
	O[i][j] = 0.0;
      for (n=1; n<=count; ++n) {
	O[i][j] -= J[ind[n]][j] * B[i][n];
      }
    }
  }

  /* compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    e[i] = 
      ralpha_dyn*w[i]*(rest[i].th - state[i].th) - 
      2.*sqrt(ralpha_dyn*w[i])/2.0 *state[i].thd;
  }
  mat_vec_mult(O,e,en);

  /* add the optimization part to the accelerations */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thdd += en[i];
  }

  return TRUE;

}


/*****************************************************************************
******************************************************************************
Function Name	: inverseKinematicsDynDecoupNoM
Date		: June 1999
   
Remarks:

        operational space controller of Khatib, but taking the C+g term
        into joint space, and no pre-mult of null space term with M
        
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

 the function updates the state by adding the appropriate joint accelerations

*****************************************************************************/
static int
inverseKinematicsDynDecoupNoM(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
			      Vector cart, iVector status, double dt, double t)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B, M, invM, JinvM;
  static iVector ind;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static double  last_t;       
  static Vector  e, en;
  //  double         ridge = 1.e-2;
  double         ridge = 1.e-8;

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    invM   = my_matrix(1,N_DOFS,1,N_DOFS);
    dJdtthd = my_vector(1,6*N_ENDEFFS);
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    JinvM  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
  }

  // compute inertia matrix and its inverse
  M = SL_inertiaMatrix(joint_state, &base_state, &base_orient, endeff);
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i)
    M[i][i] += ridge;
  my_inv_ludcmp(M, N_DOFS-N_DOFS_EST_SKIP, invM);

  /* how many contrained cartesian DOFs do we have? */
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  // compute J*inv(M) 
  for (i=1; i<=count; ++i) {
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      JinvM[ind[i]][j] = 0.0;
      for (n=1; n<=N_DOFS-N_DOFS_EST_SKIP; ++n) 
	JinvM[ind[i]][j] += J[ind[i]][n]*invM[n][j];
    }
  }

  /* build the inertia weighted pseudo-inverse according to the status
     information */
  mat_zero(P);
  for (i=1; i<=count; ++i) {
    for (j=1; j<=count; ++j) {
      for (n=1; n<=N_DOFS-N_DOFS_EST_SKIP; ++n) {
	P[i][j] += J[ind[i]][n] * JinvM[ind[j]][n];
      }
      if (i==j) 
	P[i][j] += ridge;
    }
  }

  /* invert the matrix */
  if (!my_inv_ludcmp(P, count, P)) {
    return FALSE;
  }

  /* build the B matrix, i.e., the inertia weighted pseudo-inverse */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    for (j=1; j<=count; ++j) {
      B[i][j]=0.0;
      for (n=1; n<=count; ++n) {
	B[i][j] += JinvM[ind[n]][i] * P[n][j];
      }
    }
  }

  /* need dJdt * thd  */
  for (i=1; i<=count; ++i) {
    dJdtthd[ind[i]] = 0.0;
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j)
      dJdtthd[ind[i]] += dJdt[ind[i]][j]*state[j].thd; 
  }

  /* this provides the RANGE part of the solution to the joint space acceleration */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    state[i].thdd = 0;
    for (j=1; j<=count; ++j) { 
      state[i].thdd += B[i][j] * (cart[ind[j]] - dJdtthd[ind[j]]);
    }
  }

  /**********************/
  /* the null space term */
  /**********************/

  /* compute the NULL space projection matrix; note that it is computationally
     inefficient to do this, since this is O(d^2), while all we really need is
     some matrix-vector multiplications, which are much cheaper */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    for (j=1; j<=N_DOFS-N_DOFS_EST_SKIP; ++j) {
      if (i==j) 
	O[i][j] = 1.0;
      else
	O[i][j] = 0.0;
      for (n=1; n<=count; ++n) {
	O[i][j] -= J[ind[n]][i] * B[j][n];
      }
    }
  }

  /* compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    double fac = 0.5;
    e[i] = 
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd;
  }
  mat_vec_mult(O,e,en);

  /* return this as a PD command in uff */
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i) {
    state[i].uff = en[i]*0; 
  }

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
      state[i].uff += J[ind[j]][i] * cart[ind[j]];
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
    double fac=0.1;

    e[i] = 
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd;
    vaux[i] = e[i];
  }
  mat_vec_mult(O,e,en);
  //print_mat("O",O);
  //getchar();

  /* return this as a PD command in uff */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff += en[i];
    vaux[i+7] = en[i];
  }

  // data visualization only 
  if (1) {
    // compute inertia matrix and its inverse
    M = SL_inertiaMatrix(joint_state, &base_state, &base_orient, endeff);
    my_inv_ludcmp(M, N_DOFS-N_DOFS_EST_SKIP, invM);
    mat_vec_mult_size(invM,N_DOFS-N_DOFS_EST_SKIP,N_DOFS-N_DOFS_EST_SKIP,
		      en,N_DOFS-N_DOFS_EST_SKIP,
		      en);

    for (i=1; i<=count; ++i) {
      vaux[i+14] = 0.0;
      for (j=1; j<=N_DOFS; ++j)
	vaux[i+14] += J[ind[i]][j]*en[j];
    }

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
  Function Name	: compute_target_orient
  Date		: Oct. 2005

  Remarks:

  calculates the target orientation

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
compute_target_orient(void)
{
  int i, j;
  double aux_r_norm;
  static SL_quat     last_ctarget_quat[N_ENDEFFS+1]; 
  static double      last_t;
  static int firsttime  = TRUE;
  static int skip_ticks;

  if(firsttime){ /* initialize */
    for (i=1; i<=N_ENDEFFS; ++i){
      for (j=1; j<=N_QUAT; ++j){
	last_ctarget_quat[i].q[j] = 0.0;
	last_ctarget_quat[i].qd[j] = 0.0;
	last_ctarget_quat[i].qdd[j] = 0.0;
      }
      for (j=1; j<=N_CART; ++j){
	last_ctarget_quat[i].ad[j] = 0.0;
	last_ctarget_quat[i].add[j] = 0.0;
      }
    }
    skip_ticks = 4;
    firsttime = FALSE;
  }

  for (i=1; i<=N_ENDEFFS; ++i){
    /* increase rot_amp gradually */
    ctarget_rot[i].phi = rot_amp+PI;
    ctarget_rot[i].r[_A_] = cos(2.*PI*0.25*rot_speed*(task_servo_time-start_time));
    ctarget_rot[i].r[_B_] = sin(2.*PI*0.25*rot_speed*(task_servo_time-start_time));
    ctarget_rot[i].r[_G_] = sin(2.*PI*0.25*rot_speed*(task_servo_time-start_time));
    aux_r_norm = sqrt(ctarget_rot[i].r[_A_]*ctarget_rot[i].r[_A_] + 
		      ctarget_rot[i].r[_B_]*ctarget_rot[i].r[_B_] + 
		      ctarget_rot[i].r[_G_]*ctarget_rot[i].r[_G_]);
    if (aux_r_norm < 0.0001){
      freeze();
      return FALSE;
    }

    // hack
    /*
    ctarget_rot[i].phi = PI/2.;
    ctarget_rot[i].r[_A_] = 0.0;
    ctarget_rot[i].r[_B_] = 0.0;
    ctarget_rot[i].r[_G_] = 1.0;
    */

    /* target quaternion */
    ctarget_quat[i].q[_Q0_] = cos(ctarget_rot[i].phi/2.0);
    ctarget_quat[i].q[_Q1_] = ctarget_rot[i].r[_A_]*sin(ctarget_rot[i].phi/2.0);
    ctarget_quat[i].q[_Q2_] = ctarget_rot[i].r[_B_]*sin(ctarget_rot[i].phi/2.0);
    ctarget_quat[i].q[_Q3_] = ctarget_rot[i].r[_G_]*sin(ctarget_rot[i].phi/2.0);

    /*
    ctarget_quat[i].q[_Q0_] = 0.;
    ctarget_quat[i].q[_Q1_] = 1.;
    ctarget_quat[i].q[_Q2_] = 0.;
    ctarget_quat[i].q[_Q3_] = 0.;
    */
    
    if(skip_ticks>=2){
      for (j=1; j<=N_QUAT; ++j){
	ctarget_quat[i].qd[j]  =  0.0;
	ctarget_quat[i].qdd[j] =  0.0;
      }
      skip_ticks--;
    }
    else if(skip_ticks ==1){
      for (j=1; j<=N_QUAT; ++j){
	ctarget_quat[i].qd[j]  =  (ctarget_quat[i].q[j] - last_ctarget_quat[i].q[j]) * (double)task_servo_rate;
	ctarget_quat[i].qdd[j] =  0.0;
      }
      skip_ticks--;
      }
    else{
      /* compute numerical diff. */
      for (j=1; j<=N_QUAT; ++j){
	ctarget_quat[i].qd[j]  =  (ctarget_quat[i].q[j] - last_ctarget_quat[i].q[j]) * (double)task_servo_rate;
	ctarget_quat[i].qdd[j] =  (ctarget_quat[i].qd[j] - last_ctarget_quat[i].qd[j]) * (double)task_servo_rate;
      }
    }
    
    quatDerivativesToAngVelAcc(&ctarget_quat[i]);    

    /* store last target */
    for (j=1; j<=N_QUAT; ++j){
      last_ctarget_quat[i].q[j] =   ctarget_quat[i].q[j];
      last_ctarget_quat[i].qd[j] =  ctarget_quat[i].qd[j];
      last_ctarget_quat[i].qdd[j] = ctarget_quat[i].qdd[j];
    }
    for (j=1; j<=N_CART; ++j){
      last_ctarget_quat[i].ad[j] =  ctarget_quat[i].ad[j];
      last_ctarget_quat[i].add[j] = ctarget_quat[i].add[j];
    }
  }
  
  last_t = task_servo_time;

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
