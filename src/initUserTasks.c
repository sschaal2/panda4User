/*============================================================================
==============================================================================
                      
                              initUserTasks.c
 
==============================================================================
Remarks:

         Functions needed to initialize and link user tasks for the
         simulation

============================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"
#include "SL_task_servo.h"
#include "unistd.h"
#include "limits.h"

/* global variables */

/* local variables */
static int user_tasks_initialized = FALSE;

/*****************************************************************************
******************************************************************************
Function Name	: initUserTasks
Date		: June 1999
   
Remarks:

      initialize tasks that are not permanently linked in the simulation
      This replaces the <ltask facility in vxworks -- just that we cannot
      do on-line linking in C.

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
void
initUserTasks(void)

{
  extern void add_sm_task();
  extern void add_gravcomp_task();
  extern void add_test_task();
  extern void add_impedance_test_task();
  extern void add_cilantro_track_task();


  add_gravcomp_task();
  add_sm_task();
  add_test_task();
  add_impedance_test_task();
  add_cilantro_track_task();

  //sprintf(initial_user_command,"go0");

  char cwd[PATH_MAX];
  getcwd(cwd, sizeof(cwd));
  printf("Current working dir: %s\n", cwd);  
}

 
