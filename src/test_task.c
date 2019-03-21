/*============================================================================
==============================================================================
                      
                              test_task.c
 
==============================================================================
Remarks:

       test task

============================================================================*/

/* vxWorks includes */
#include "SL_system_headers.h"

/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"

/* defines */

/* local variables */
static double     time_step;
static double     start_time=0;
static double     task_time;
static int        ax= _X_;
static int        my_dof = 4;
static double     force = 0.0;
static int        torque_flag = FALSE;

/* global functions */
void add_test_task(void);

/* local functions */
static int  init_test_task(void);
static int  run_test_task(void);
static int  change_test_task(void);

/*****************************************************************************
******************************************************************************
Function Name	: add_test_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_test_task( void )

{
  int i, j;
  
  addTask("Test Task", init_test_task, 
	  run_test_task, change_test_task);

}    

/*****************************************************************************
******************************************************************************
  Function Name	: init_test_task
  Date		: Dec. 1997

   Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int 
init_test_task(void)
{
  int    j,i;
  int    ans;

  get_int("Which axis?",ax,&ax);

  ans = 999;
  while (ans == 999) {
    if (!get_int("Enter 1 to start or anthing else to abort ...",ans,&ans))
      return FALSE;
  }
  
  if (ans != 1) 
    return FALSE;
  
  task_time = 0.0;
  start_time = task_servo_time;
  
  return TRUE;

}

/*****************************************************************************
******************************************************************************
  Function Name	: run_test_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
run_test_task(void)
{
  int j, i;
  double pos[N_CART+1];
  double rot[N_CART+1];


  task_time += 1./(double)task_servo_rate;
  
  if (torque_flag)
    uext_sim[my_dof].t[ax] = force;
  else
    uext_sim[my_dof].f[ax] = force;

  sendUextSim();

  return TRUE;

}

/*****************************************************************************
******************************************************************************
  Function Name	: change_test_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int 
change_test_task(void)
{
  int j, i;

  get_int("Which axis? X=1, Y=2, Z=3",ax,&ax);
  get_int("Which DOF?",my_dof,&my_dof);
  get_double("Which force?",force,&force);
  get_int("Force=0 Torque=1",torque_flag,&torque_flag);

  return TRUE;

}

