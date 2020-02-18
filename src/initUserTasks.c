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
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "unistd.h"
#include "limits.h"

/* global variables */

/* local variables */
static int user_tasks_initialized = FALSE;
int no_user_interaction_flag = FALSE;

// local functions
static void runInsertions(void);


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

  addToMan("run_insertions","runs several insertion state-machines",runInsertions);

  //sprintf(initial_user_command,"go0");

}

 
/*****************************************************************************
******************************************************************************
Function Name   : runTrials
Date            : Sept 2006
Remarks:

       runs several insertion state machines in concatention


******************************************************************************
Paramters:  (i/o = input/output)

   none

*****************************************************************************/
static void
runInsertions(void) {

  int i;
  int n_iterations = 1;
  extern int sm_run_table;
  extern char sm_file_name[];

  get_int("How many iterations?",n_iterations,&n_iterations);

  for (i=1; i<=n_iterations+1; ++i) {

    while (strcmp(current_task_name,NO_TASK) != 0)
      sleep(1);

    if (i==n_iterations+1)
      break;

    no_user_interaction_flag = TRUE;
    sm_run_table = TRUE;
    if (i%6 == 1)
      strcpy(sm_file_name,"powerplug_JT_1.sm");
    else if (i%6 == 2)
      strcpy(sm_file_name,"hdmi_JT_1.sm");
    else if (i%6 == 3)
      strcpy(sm_file_name,"usb_JT_1.sm");
    else if (i%6 == 4)
      strcpy(sm_file_name,"hdmi_JT_2.sm");
    else if (i%6 == 5)
      strcpy(sm_file_name,"usb_JT_2.sm");
    else if (i%6 == 0)
      strcpy(sm_file_name,"powerplug_JT_2.sm");


    setTaskByName("State Machine Task");

    while (strcmp(current_task_name,NO_TASK) == 0)
      sleep(1);
    
    printf("========= Restart %d ==========\n",n_iterations);

  }

  printf("\n\n All done\n");

  no_user_interaction_flag = FALSE;
}
