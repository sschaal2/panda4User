/*============================================================================
==============================================================================
                      
                              sm_controllers.c
 
==============================================================================
Remarks:

      implements a variety of different controllers to be used for 
      sm_task.c

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

// local variables

static int sm_controllers_initialized = FALSE;

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

static double controller_gain_th[N_DOFS+1];
static double controller_gain_thd[N_DOFS+1];
static double controller_gain_int[N_DOFS+1];

static double cref_integral[N_ENDEFFS*6+1];


// global functions

void
init_sm_controllers( void );

int
cartesianImpedanceSimpleJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double *gain_x_scale,
			   double *gain_xd_scale,
			   double *gain_a_scale,
			   double *gain_ad_scale);

int
cartesianImpedanceModelJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double *gain_x_scale,
			   double *gain_xd_scale,
			   double *gain_a_scale,
			   double *gain_ad_scale);

// local functions 
static int computePseudoInverseAndNullSpace(iVector status, Matrix Jprop, int *nr, Matrix Jhash, Matrix Nproj);
static int init_filters(void);
static int computeInertiaWeightedPseudoInverseAndNullSpace(iVector status, Matrix Jprop, int *nr, Matrix Jhash, Matrix Nproj,
							   Matrix JTJbar, Vector dJdtthd);



/*****************************************************************************
******************************************************************************
Function Name	: init_sm_controllers
Date		: Feb 2019
Remarks:

initializes various variable relevant for the sm controllers

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
init_sm_controllers( void )
{
  int i, j;
  char string[100];
  static int firsttime = TRUE;

  
  if (firsttime) {
    firsttime = FALSE;
    
  // initialize the filters 
    init_filters();
    for (i=1; i<=N_DOFS; ++i) 
      fthdd[i].cutoff = 5;
  
    // read the control gains 
    if (!read_gains(config_files[GAINS],controller_gain_th, 
		    controller_gain_thd, controller_gain_int))
      return;

    sm_controllers_initialized = TRUE;
    
  }

  // zero the filters 
  for (i=1; i<=N_DOFS; ++i) 
    for (j=0; j<=FILTER_ORDER; ++j)
      fthdd[i].raw[j] = fthdd[i].filt[j] = 0;
  
  // zero out intergrator
  bzero((char *)&cref_integral,sizeof(cref_integral));

  
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

     \param[in]      cdes    : the desired cartesian trajectory
     \param[in]      cdes_orient : the desired orientation cartesian trajectory
     \param[in,out]  state   : the state of the robot (given as a desired state; note
                               that this desired state coincides with the true state of
                               the robot, as the joint space PD controller is replaced
                               by the Cartesian controller)
     \param[in]      rest    : the optimization posture
     \param[in]      status  : which rows to use from the Jacobian
     \param[in]      gain_integral: integral gain: set to zero if not used
     \param[in]      gain_x_scale : scales the default pos gains up-or-down (1=no scale)
     \param[in]      gain_xd_scale: scales the default vel gains up-or-down (1=no scale)
     \param[in]      gain_a_scale : scales the default orient gains up-or-down (1=no scale)
     \param[in]      gain_ad_scale: scales the default orient vel gains up-or-down (1=no scale)

 the function updates the state by adding the appropriate feedforward torques

*****************************************************************************/
int
cartesianImpedanceSimpleJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			   SL_OJstate *rest, iVector status,
			   double  gain_integral,
			   double *gain_x_scale,
			   double *gain_xd_scale,
			   double *gain_a_scale,
			   double *gain_ad_scale)

{
  double         default_gain   = 800;  // was 450
  double         default_gain_orient = 40;  // was 40
  
  double         corient_error[N_CART+1];
  double         cref[N_ENDEFFS*6+1];
  static double  cref_integral[N_ENDEFFS*6+1];
  double         q_rel[N_QUAT+1];
  double         q_rel_angle;
  double         log_q_mult;

  int            i,j,n,m;
  int            nr = 0;
  int            count = 0;
  static Matrix  Jprop, Jhash, Nproj;
  static int     firsttime = TRUE;
  static Vector  e, en;
  

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    Jhash  = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    Nproj  = my_matrix(1,N_DOFS,1,N_DOFS);
    Jprop  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);

    if (!sm_controllers_initialized)
      init_sm_controllers();      
    
  }

  // if no intergral controller, zero intergrator state
  if (gain_integral == 0)
    bzero((char *)&cref_integral,sizeof(cref_integral));

  // compute orientation error term for quaterion feedback control
  quatRelative(cart_orient[HAND].q,cdes_orient[HAND].q,q_rel);
  for (i=1; i<=N_CART; ++i)
    corient_error[i] = -q_rel[_Q0_+i];

  q_rel_angle = acos(q_rel[_Q0_]);
  log_q_mult = q_rel_angle/(sqrt(vec_mult_inner_size(corient_error,corient_error,N_CART))+1.e-6);
  
  // prepare the impdance controller, i.e., compute operational
  // space force command
  
  for (j= _X_; j<= _Z_; ++j) {
    
    if (status[j]) {
      ++count;
      
      cref_integral[count] += gain_integral * (cdes[HAND].x[j]  - cart_state[HAND].x[j]);
      
      cref[count] = cref_integral[count] +
	(cdes[HAND].xd[j]  - cart_state[HAND].xd[j]) * 2.*sqrt(default_gain) * gain_xd_scale[j] +
	(cdes[HAND].x[j]  - cart_state[HAND].x[j]) * default_gain * gain_x_scale[j];
    }
  }
  
  for (j= _A_; j<= _G_ ; ++j) { /* orientation */
    if (status[N_CART + j]) {
      ++count;
      
      cref_integral[count] +=  - 0.1 * log_q_mult * corient_error[j] * gain_integral;
      
      cref[count] = cref_integral[count] +
	(cdes_orient[HAND].ad[j] - cart_orient[HAND].ad[j]) *0.025 * 2.0 * sqrt(default_gain_orient) * gain_ad_scale[j] - 
	log_q_mult * corient_error[j] * default_gain_orient * gain_a_scale[j]; 
    }
  }
  

    // build the desired state and uff
  for (i=1; i<=N_DOFS; ++i) {
    state[i].th   = joint_state[i].th;  // zeros out P control on motor_servo
    state[i].thd  = joint_state[i].thd; // zeros out D control on motor_servo
    state[i].thdd = 0.0;
    state[i].uff  = 0.0;
  }

  // inverse dynamics: feedback linerization of C+G term (note thdd=0)
  SL_InvDyn(joint_state,state,endeff,&base_state,&base_orient);

  // get the properly sized Jacobian + pseudoInverse + Nullspace Projector
  computePseudoInverseAndNullSpace(status, Jprop, &nr, Jhash, Nproj);
  
  // the simple cartesion impedance controller only uses J-trans 
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=nr; ++j) {
      state[i].uff += Jprop[j][i] * cref[j];
    }
  }

  // compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS; ++i) {
    double fac=1.0;
    e[i] = rest[i].w*(
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd);
    //printf("%d.%f (w=%f)\n",i,e[i],rest[i].w);    
  }
  mat_vec_mult(Nproj,e,en);

  /* add this as a PD command to uff */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff += en[i];
    //printf("%d.%f\n",i,en[i]);    
  }

  return TRUE;

}

/*****************************************************************************
******************************************************************************
Function Name	: cartesianImpedanceModelJt
Date		: March 2019
   
Remarks:

        computes a Jacobian transpose impedance controller with
        dynamics model, but without inertial shaping
        
******************************************************************************
Paramters:  (i/o = input/output)

     \param[in]      cdes    : the desired cartesian trajectory
     \param[in]      cdes_orient : the desired orientation cartesian trajectory
     \param[in,out]  state   : the state of the robot (given as a desired state; note
                               that this desired state coincides with the true state of
                               the robot, as the joint space PD controller is replaced
                               by the Cartesian controller)
     \param[in]      rest    : the optimization posture
     \param[in]      status  : which rows to use from the Jacobian
     \param[in]      gain_integral: integral gain: set to zero if not used
     \param[in]      gain_x_scale : scales the default pos gains up-or-down (1=no scale)
     \param[in]      gain_xd_scale: scales the default vel gains up-or-down (1=no scale)
     \param[in]      gain_a_scale : scales the default orient gains up-or-down (1=no scale)
     \param[in]      gain_ad_scale: scales the default orient vel gains up-or-down (1=no scale)

 the function updates the state by adding the appropriate feedforward torques

*****************************************************************************/
int
cartesianImpedanceModelJt(SL_Cstate *cdes, SL_quat *cdes_orient, SL_DJstate *state,
			  SL_OJstate *rest, iVector status,
			  double  gain_integral,
			  double *gain_x_scale,
			  double *gain_xd_scale,
			  double *gain_a_scale,
			  double *gain_ad_scale)

{
  double         default_gain   = 800;  // was 450
  double         default_gain_orient = 40;  // was 40
  
  double         corient_error[N_CART+1];
  double         cref[N_ENDEFFS*6+1];
  static double  cref_integral[N_ENDEFFS*6+1];
  double         q_rel[N_QUAT+1];
  double         q_rel_angle;
  double         log_q_mult;

  int            i,j,n,m;
  int            nr = 0;
  int            count = 0;
  static Matrix  Jprop, Jhash, Nproj, JTJbar;
  static Vector  dJdtthd;
  static int     firsttime = TRUE;
  static Vector  e, en;
  

  /* initialization of static variables */
  if (firsttime) {
    firsttime = FALSE;
    e      = my_vector(1,N_DOFS);
    en     = my_vector(1,N_DOFS);
    Jhash  = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    Nproj  = my_matrix(1,N_DOFS,1,N_DOFS);
    Jprop  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
    JTJbar = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    dJdtthd= my_vector(1,6*N_ENDEFFS);

    if (!sm_controllers_initialized)
      init_sm_controllers();      
    
  }

  // if no intergral controller, zero intergrator state
  if (gain_integral == 0)
    bzero((char *)&cref_integral,sizeof(cref_integral));

  // compute orientation error term for quaterion feedback control
  quatRelative(cart_orient[HAND].q,cdes_orient[HAND].q,q_rel);
  for (i=1; i<=N_CART; ++i)
    corient_error[i] = -q_rel[_Q1_+i-1];

  q_rel_angle = acos(q_rel[_Q0_]);
  log_q_mult = q_rel_angle/(sqrt(vec_mult_inner_size(corient_error,corient_error,N_CART))+1.e-6);
  
  // prepare the impdance controller, i.e., compute operational
  // space force command
  
  for (j= _X_; j<= _Z_; ++j) {
    
    if (status[j]) {
      ++count;
      
      cref_integral[count] += gain_integral * (cdes[HAND].x[j]  - cart_state[HAND].x[j]);
      
      cref[count] = cref_integral[count] +
	(cdes[HAND].xd[j]  - cart_state[HAND].xd[j]) * 2.*sqrt(default_gain) * gain_xd_scale[j] +
	(cdes[HAND].x[j]  - cart_state[HAND].x[j]) * default_gain * gain_x_scale[j];
    }
  }
  

  for (j= _A_; j<= _G_ ; ++j) { /* orientation */
    if (status[N_CART + j]) {
      ++count;
      
      cref_integral[count] +=  - 0.1 * log_q_mult * corient_error[j] * gain_integral;
      
      cref[count] = cref_integral[count] +
	(cdes_orient[HAND].ad[j] - cart_orient[HAND].ad[j]) *0.025 * 2.0 * sqrt(default_gain_orient) * gain_ad_scale[j] - 
	log_q_mult * corient_error[j] * default_gain_orient * gain_a_scale[j]; 
    }
  }
  

    // build the desired state and uff
  for (i=1; i<=N_DOFS; ++i) {
    state[i].th   = joint_state[i].th;  // zeros out P control on motor_servo
    state[i].thd  = joint_state[i].thd; // zeros out D control on motor_servo
    state[i].thdd = 0.0;
    state[i].uff  = 0.0;
  }

  // inverse dynamics: feedback linerization of C+G term (note thdd=0)
  SL_InvDyn(joint_state,state,endeff,&base_state,&base_orient);

  // get the properly sized Jacobian + pseudoInverse + Nullspace Projector
  computeInertiaWeightedPseudoInverseAndNullSpace(status, Jprop, &nr, Jhash, Nproj, JTJbar, dJdtthd);
  
  // the simple cartesion impedance controller only uses J-trans 
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=nr; ++j) {
      state[i].uff += Jprop[j][i] * cref[j];
    }
  }

  // compute the PD term for the Null space  */
  for (i=1; i<=N_DOFS; ++i) {
    double fac=0.1;
    e[i] = rest[i].w*(
      fac*controller_gain_th[i]*(rest[i].th - state[i].th) - 
      sqrt(fac)*controller_gain_thd[i] *state[i].thd);
  }
  mat_vec_mult(Nproj,e,en);

  // add this as a PD command to uff 
  for (i=1; i<=N_DOFS; ++i) {
    state[i].uff += en[i];
    //printf("%d.%f\n",i,en[i]);
  }

  // compute the models based feedforward term
  
  // re-use "cref" and "en" for a new purpose
  count = 0;
  for (j= _X_; j<= _Z_; ++j) {
      if (status[j]) {
      ++count;
      cref[count] = cdes[HAND].xdd[j] - dJdtthd[count];
    }
  }

  for (j= _A_; j<= _G_ ; ++j) {
    if (status[N_CART + j]) {
      ++count;
      cref[count] = cdes_orient[HAND].add[j] - dJdtthd[count];;
    }
  }

  mat_vec_mult_size(JTJbar,N_DOFS,count,cref,count,en);

  // add this as a feedforward command to uff 
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
Function Name	: SL_inertiaMatrix
Date		: March 2019
   
Remarks:

        computes the RBD inertia matrix with composite dynammics of
        Featherstone
        
******************************************************************************
Paramters:  (i/o = input/output)

    \param[in]     lstate  : the desired state
    \param[in]     cbase   : the position state of the base
    \param[in]     obase   : the orientational state of the base
    \param[in]     endeff  : the endeffector parameters

 the pointer to the rbdM matrix is returned

*****************************************************************************/
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

/*****************************************************************************
******************************************************************************
Function Name	: computePseudoInverseAndNullSpace
Date		: March 2019
   
Remarks:

        computes the pseudo-inverse and null space projector
        
******************************************************************************
Paramters:  (i/o = input/output)

     \param[in]  status  : which rows to use from the Jacobian
     \param[out] Jprop   : Jacobian with only relevant rows
     \param[out] nr      : number of rowns in Jprop
     \param[out] Jhash   : pseudo inverse
     \param[out] Nproj   : null space projection matrix
 

*****************************************************************************/
static int
computePseudoInverseAndNullSpace(iVector status, Matrix Jprop, int *nr, Matrix Jhash, Matrix Nproj)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B, Jred;
  static iVector ind;
  static int     firsttime = TRUE;
  double         ridge = 1.e-8;

  // initialization of static variables with max possible size
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    Jred   = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
  }

  // how many contrained cartesian DOFs do we have?
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  // the reduced Jacobian
  mat_zero(Jred);
  for (i=1; i<=count; ++i) {
    for (n=1; n<=N_DOFS; ++n) {
      Jred[i][n] = J[ind[i]][n];
    }
  }

  // build the pseudo-inverse according to the status information
  mat_zero(P);
  mat_mult_normal_transpose_size(Jred,count,N_DOFS,Jred,count,N_DOFS,P);
  for (i=1; i<=count; ++i) 
    P[i][i] += ridge;

  // invert the matrix 
  if (!my_inv_ludcmp(P, count, P)) {
    return FALSE;
  }

  // build the B matrix, i.e., the pseudo-inverse
  mat_zero(B);
  mat_mult_transpose_normal_size(Jred,count,N_DOFS,P,count,count,B);


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
	O[i][j] -= B[i][n] * Jred[n][j];
      }

      if (i!=j)
	O[j][i] = O[i][j];
    }
  }


  // provide points to result matrices
  mat_equal_size(Jred,count,N_DOFS,Jprop);
  mat_equal_size(B,N_DOFS,count,Jhash);
  mat_equal_size(O,N_DOFS,N_DOFS,Nproj);
  *nr    = count;

  return TRUE;

}

/*****************************************************************************
******************************************************************************
Function Name	: computeInertiaWeightedPseudoInverseAndNullSpace
Date		: March 2019
   
Remarks:

        computes the pseudo-inverse and null space projector with the
        invertia weighting, i.e., as needed in proper operational space
        control
        
******************************************************************************
Paramters:  (i/o = input/output)

     \param[in]  status  : which rows to use from the Jacobian
     \param[out] Jprop   : Jacobian with only relevant rows
     \param[out] nr      : number of rowns in Jprop
     \param[out] Jhash   : pseudo inverse
     \param[out] Nproj   : null space projection matrix
 

*****************************************************************************/
static int
computeInertiaWeightedPseudoInverseAndNullSpace(iVector status, Matrix Jprop, int *nr, Matrix Jhash, Matrix Nproj, Matrix JTJbar, Vector dJdtthd)
{
  
  int            i,j,n,m;
  int            count;
  static Matrix  O, P, B, Jred, M, invM, JinvM;
  static iVector ind;
  static int     firsttime = TRUE;
  double         ridge = 1.e-8;

  // initialization of static variables with max possible size
  if (firsttime) {
    firsttime = FALSE;
    P      = my_matrix(1,6*N_ENDEFFS,1,6*N_ENDEFFS);
    B      = my_matrix(1,N_DOFS,1,6*N_ENDEFFS);
    ind    = my_ivector(1,6*N_ENDEFFS);
    O      = my_matrix(1,N_DOFS,1,N_DOFS);
    Jred   = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
    invM   = my_matrix(1,N_DOFS,1,N_DOFS);
    JinvM  = my_matrix(1,6*N_ENDEFFS,1,N_DOFS);
  }

  // compute inertia matrix and its inverse
  M = SL_inertiaMatrix(joint_state, &base_state, &base_orient, endeff);
  for (i=1; i<=N_DOFS-N_DOFS_EST_SKIP; ++i)
    M[i][i] += ridge;
  my_inv_ludcmp(M, N_DOFS-N_DOFS_EST_SKIP, invM);

  // how many contrained cartesian DOFs do we have?
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  // the reduced Jacobian and dJdtthd
  mat_zero(Jred);
  vec_zero(dJdtthd);
  for (i=1; i<=count; ++i) {
    for (n=1; n<=N_DOFS; ++n) {
      Jred[i][n] = J[ind[i]][n];
      dJdtthd[i] += dJdt[ind[i]][n]*joint_state[n].thd;
    }
  }

  // compute J*inv(M)
  mat_mult_size(Jred,count,N_DOFS,invM,N_DOFS,N_DOFS,JinvM);

  /* build the inertia weighted pseudo-inverse according to the status
     information */
  mat_zero(P);
  mat_mult_normal_transpose_size(JinvM,count,N_DOFS,Jred,count,N_DOFS,P);
  for (i=1; i<=count; ++i)
    P[i][i] += ridge;

  /* invert the matrix */
  if (!my_inv_ludcmp(P, count, P)) {
    return FALSE;
  }

  /* build the B matrix, i.e., the inertia weighted pseudo-inverse */
  mat_mult_transpose_normal_size(JinvM,count,N_DOFS,P,count,count,B);


  /* build the J'*Jbar matrix, i.e., the inertia weighted pseudo-inverse left multipled by the inertia matrix*/
  mat_mult_transpose_normal_size(Jred,count,N_DOFS,P,count,count,JTJbar);


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
	O[i][j] -= Jred[n][i] * B[j][n];
      }
    }
  }

  // provide points to result matrices
  mat_equal_size(Jred,count,N_DOFS,Jprop);
  mat_equal_size(B,N_DOFS,count,Jhash);
  mat_equal_size(O,N_DOFS,N_DOFS,Nproj);
  *nr    = count;

  return TRUE;

}


