/*============================================================================
  ==============================================================================

  data_collection_lib.cpp

  ==============================================================================
  Remarks:

  This is a lib for running an additional thread that communicates
  with a data collection process running on the same machine.

  ============================================================================*/
// system includes
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <iterator>
#include <mutex>
#include <thread>
#include <zmq.hpp>
#include <list>
#include <json/json.h>
#include <stdexcept>

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"
#include "SL_task_servo.h"
#include "SL_unix_common.h"
#include "SL_common.h"
#include "SL_vx_wrappers.h"
#include "utility.h"
#include "data_collection_lib.h"

// local variables
static pthread_t       com_thread;   // thread for TCP connection

// global varibales
int    run_com_thread_flag = FALSE;
int    robot_ready_for_data_col_image = FALSE;
int    robot_ready_for_data_col_state = FALSE;
int    robot_ready_for_data_col_label = FALSE;
int    sm_ready_for_pose_delta_prediction = FALSE;
int    global_sample_id = 0;
double reference_state_pose_delta_x_adjusted[N_CART+1];
double reference_state_pose_delta_q_adjusted[N_QUAT+1];
double predicted_reference_state_pose_delta_x_inverse[N_CART+1];
double predicted_reference_state_pose_delta_q_inverse[N_QUAT+1];
double max_allowed_abs_diff_predicted_reference_state_pose_delta_x = 0.007; // meters
double max_allowed_abs_diff_predicted_reference_state_pose_delta_q = 5.0;   // degree
double max_abs_diff_predicted_reference_state_pose_delta_x = 0.0;
double max_abs_diff_predicted_reference_state_pose_delta_q = 0.0;
int    continue_pose_correction = FALSE;

// external variables
extern double reference_state_pose_x[];
extern double reference_state_pose_q[];
extern double reference_state_pose_x_base[];
extern double reference_state_pose_q_base[];
extern double reference_state_pose_delta_x_table[][N_CART+1];
extern double reference_state_pose_delta_q_table[][N_QUAT+1];
extern int    current_state_pose_delta;
extern double pos_error_vector[];
extern double orient_error_quat[];
extern SL_Cstate ctarget[];
extern SL_quat ctarget_orient[];

// local functions
void add_joints_state(Json::Value &state);
void add_endeffector_cart(Json::Value &state);
void add_endeffector_orient(Json::Value &state);
void add_endeffector_calculated_force_torque(Json::Value &state);
void add_experiment_data(Json::Value &state);
void spawnComThread(void);
void *ComThread(void *);
static void   continuePoseCorrection(void);

extern "C" int  initDataCollection(void);
extern "C" void triggerPoseDeltaPrediction(void);
extern "C" void triggerDataCollection(void);
extern "C" int  checkPoseDeltaPrediction(void);
extern "C" int  checkDataCollection(void);

/*!*****************************************************************************
*******************************************************************************
\note  initDataCollection
\date  Oct 2020

\remarks

initializes deep learning data collection for pose correction

*******************************************************************************
Function Parameters: [in]=input,[out]=output

return TRUE/FALSE for success/failure

******************************************************************************/
int
initDataCollection(void)
{
  int counter = 0;

  if (!run_com_thread_flag)  {

    addToMan("cc","sets flags to execute pose correction",continuePoseCorrection); 
    
    // spawn TCP data-logger com thread
    printf("Spawn Com Thread...\n");
    spawnComThread();
    
    // wait for first handshake to set the next sample-id
    printf("Wait for data-collection-client to set sample_id...\n");
    while(global_sample_id == 0){
      if (++counter > 5000)
	return FALSE;
      
      // just wait 1000 ticks
      taskDelay(1000);
    }
  }
  
  return TRUE;

}

/*!*****************************************************************************
*******************************************************************************
\note  continuePoseCorrection
\date  Oct 2020

\remarks

sets the flag for allowing to execute pose correction

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
static void
continuePoseCorrection(void)
{

  continue_pose_correction = TRUE;

}

/*!*****************************************************************************
*******************************************************************************
\note  triggerDataCollection
\date  Oct 2020

\remarks

sets the flag for data collections for the communication thread

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
triggerDataCollection(void)
{
  char msg[100];
  
  sprintf(msg,"\nTrigger Data Collection %d\n",global_sample_id);
  logMsg(msg,0,0,0,0,0,0);

  robot_ready_for_data_col_image = TRUE;
  robot_ready_for_data_col_state = TRUE;

}

/*!*****************************************************************************
*******************************************************************************
\note  checkDataCollection
\date  Oct 2020

\remarks

checks the flag for data collections for completion of the task

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
checkDataCollection(void)
{

  if (!robot_ready_for_data_col_image && !robot_ready_for_data_col_state)
    return TRUE;

  return FALSE;

}

/*!*****************************************************************************
*******************************************************************************
\note  triggerPoseDeltaPrediction
\date  Oct 2020

\remarks

sets the flag for pose delta prediction for the communication thread

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
triggerPoseDeltaPrediction(void)
{

  robot_ready_for_data_col_image = TRUE;
  robot_ready_for_data_col_state = TRUE;
  sm_ready_for_pose_delta_prediction = TRUE;

}

/*!*****************************************************************************
*******************************************************************************
\note  checkPoseDeltaPrediction
\date  Oct 2020

\remarks

checks the flag for pose delta prediction for completion of task

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
int
checkPoseDeltaPrediction(void)
{

  if (!sm_ready_for_pose_delta_prediction && !robot_ready_for_data_col_image
      && !robot_ready_for_data_col_state)
    return TRUE;

  return FALSE;

}

/*!*****************************************************************************
*******************************************************************************
\note  spawnCommunicationThread
\date  Mai 2019

\remarks

spawns off a thread to do non-real-time communication with data_logger

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
spawnComThread(void)
{
  int err = 0;
  int rc;
  pthread_attr_t pth_attr;
  size_t stack_size = 0;

  err = pthread_attr_init(&pth_attr);
  pthread_attr_getstacksize(&pth_attr, &stack_size);
  double reqd = 1024*1024*8;
  if (stack_size < reqd)
    pthread_attr_setstacksize(&pth_attr, reqd);

  // initialize a thread for blocking gripper commands
  run_com_thread_flag = TRUE;
  if ((rc=pthread_create( &com_thread, &pth_attr, ComThread, NULL)))
    printf("com_thread_create returned with %d\n",rc);

}

/*!*****************************************************************************
*******************************************************************************
\note  ComThread
\date  Mai 2019

\remarks

non-realtime thread for data-logger communication

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void *
ComThread(void *)
{

  //  Prepare our context and socket
  zmq::context_t context (1);
  zmq::socket_t socket (context, ZMQ_REP);
  socket.bind ("tcp://*:5555");

  // a json fast writer for data
  Json::FastWriter json_fastwriter;

  // conformation code is prefix 'done:' + full request code
  std::string confirm_prefix = "done:";

  // definition of request codes
  std::string set_sample_id_req               = "set_sample_id:";
  std::string robot_ready_req                 = "confirm_robot_ready:";
  std::string prep_data_req                   = "prep_cont_data:";
  std::string send_data_req                   = "send_cont_data:";
  std::string prep_result_req                 = "prep_result:";
  std::string send_result_req                 = "send_result:";
  std::string receive_pose_delta_pred_request = "confirm_ready_for_pose_delta_pred:";
  char num;

  while (run_com_thread_flag) {
    zmq::message_t request;
    
    //  Wait for next request from client and cast it to string
    socket.recv (&request, 0);
    std::string req = std::string(static_cast<char*>(request.data()),
				  request.size());
    
    // check if a set sample id request
    if (req.std::string::find(set_sample_id_req) != std::string::npos){
      // extract the sample id
      std::string sample_id = req.substr (set_sample_id_req.length(),
                                          req.length());
      
      std::cout << "Received task to set_sample_id to: "
                << sample_id
                << std::endl;
      
      // here we set the global sample id
      global_sample_id = std::stoi(sample_id);
      
      //  Send reply back to client and check if correct by composition
      std::string confirmation = confirm_prefix +
	set_sample_id_req + std::to_string(global_sample_id);
      
      // init the reply message
      zmq::message_t reply (confirmation.length());
      // compy content into reply
      memcpy ((void *) reply.data (), confirmation.c_str(),
	      confirmation.length());
      // send confirmation
      socket.send (reply);
      std::cout << "Confirmed: set sample_id. ID: "<< global_sample_id << std::endl;
      
      // robot ready confirmation
    }else if (req.std::string::find(robot_ready_req) != std::string::npos){
      // extract the sample id
      std::string sample_id = req.substr (robot_ready_req.length(),
                                          req.length());
      
      // here we should check that and wait for it
      while (robot_ready_for_data_col_image == FALSE){
	// just wait
	taskDelay(ns2ticks(10000000000));
      }
      
      std::cout << "      ---- Start for ID:" << global_sample_id << " ----"
                << std::endl;
      
      
      std::cout << "      Received task to confirm robot ready for images of sample_id: "
                << sample_id
                << std::endl;
      
      //  Send reply back to client and check if correct by composition
      std::string confirmation = confirm_prefix +
	robot_ready_req + std::to_string(global_sample_id);
      
      // init the reply message
      zmq::message_t reply (confirmation.length());
      // copy content into reply
      memcpy ((void *) reply.data (), confirmation.c_str(),
	      confirmation.length());
      // send confirmation
      socket.send (reply);
      std::cout << "      Confirmed: robot ready. ID: "<< global_sample_id << std::endl;
      
      // robot data prep and send confirmation
    }else if (req.std::string::find(prep_data_req) != std::string::npos){
      // extract the sample id
      std::string sample_id = req.substr (prep_data_req.length(),
                                          req.length());
      
      std::cout << "      Received task to prepare data for sample_id: "
                << sample_id
                << std::endl;
      
      // here we should check that and wait for it
      while (robot_ready_for_data_col_state == FALSE){
	// just wait
	taskDelay(ns2ticks(10000000000));
      }
      robot_ready_for_data_col_state = FALSE;
      
      //  Send reply back to client and check if correct by composition
      std::string confirmation = confirm_prefix +
	prep_data_req + std::to_string(global_sample_id);
      
      // init the reply message
      zmq::message_t reply (confirmation.length());
      // compy content into reply
      memcpy ((void *) reply.data (), confirmation.c_str(),
	      confirmation.length());
      
      // send confirmation
      socket.send (reply);
      std::cout << "      Confirmed: data ready. ID: "<< global_sample_id << std::endl;
      
      // we reive the send data request
      socket.recv (&request, 0);
      std::string req = std::string(static_cast<char*>(request.data()),
				    request.size());
      // make sure it is requested
      if (req != "send_cont_data"){
        std::cout << "      DATA SEND REQUEST FAILED!!!! ID: "<< global_sample_id
                  << std::endl;
        throw std::invalid_argument("see above");
      }
      
      // collect the state
      Json::Value state;
      add_endeffector_orient(state);
      add_endeffector_cart(state);
      add_joints_state(state);
      add_endeffector_calculated_force_torque(state);
      add_experiment_data(state);
      state["sample_id"] = global_sample_id;
      
      // get json to sendable format
      std::string state_string = json_fastwriter.write(state);
      
      // init the reply message
      zmq::message_t data (state_string.size());
      // compy content into reply
      memcpy ((void *) data.data (), state_string.c_str(),
	      state_string.size());
      
      // send the data
      socket.send (data);
      std::cout << "      Send data for ID: "<< global_sample_id << std::endl;
      
      // the reset implies we are done wiht the task
      robot_ready_for_data_col_image = FALSE;
      
      // process result prep and send confirmation
    }else if (req.std::string::find(prep_result_req) != std::string::npos){
      // extract the sample id
      std::string sample_id = req.substr (prep_result_req.length(),
                                          req.length());
      
      std::cout << "      Received task to prepare result for sample_id: "
                << sample_id
                << std::endl;
      
      // here we should check that and wait for it
      while (robot_ready_for_data_col_label == FALSE){
        // just wait
        taskDelay(ns2ticks(10000000000));
      }
      
      //  Send reply back to client and check if correct by composition
      std::string confirmation = confirm_prefix +
	prep_result_req + std::to_string(global_sample_id);
      
      // init the reply message
      zmq::message_t reply (confirmation.length());
      // compy content into reply
      memcpy ((void *) reply.data (), confirmation.c_str(),
	      confirmation.length());
      // send confirmation
      socket.send (reply);
      std::cout << "      Confirmed: result ready. ID: "<< global_sample_id << std::endl;
      
      // we reive the send data request
      socket.recv (&request, 0);
      std::string req = std::string(static_cast<char*>(request.data()),
				    request.size());
      // make sure it is requested
      if (req != "send_result"){
        std::cout << "      DATA SEND REQUEST FAILED!!!! ID: "<< global_sample_id
                  << std::endl;
        throw std::invalid_argument("see above");
      }
      
      // collect the state/result
      Json::Value result;
      add_endeffector_orient(result);
      add_endeffector_cart(result);
      add_joints_state(result);
      add_endeffector_calculated_force_torque(result);
      
      result["sample_id"] = global_sample_id;
      
      // THIS IS NOT IMPLEMENTED YET!!!!!
      result["result"] = "NONE";
      // THIS IS NOT IMPLEMENTED YET!!!!!
      
      // get json to sendable format
      std::string result_string = json_fastwriter.write(result);
      
      // init the reply message
      zmq::message_t result_msg (result_string.size());
      // compy content into reply
      memcpy ((void *) result_msg.data (), result_string.c_str(),
	      result_string.size());
      
      // send the result
      socket.send (result_msg);
      std::cout << "      Send result for ID: "<< global_sample_id << std::endl;
      
      // the reset implies we are done with the task
      robot_ready_for_data_col_label = FALSE;
      
      // handshake for getting the pose delta estimation
    }else if (req.std::string::find(receive_pose_delta_pred_request) != std::string::npos){

      // reset the compensation
      for (int i=1; i<=N_CART; ++i) {
	predicted_reference_state_pose_delta_x_inverse[i] = 0.0;
      }
      for (int i=1; i<=N_QUAT; ++i) {
	predicted_reference_state_pose_delta_q_inverse[i] = 0.0;
      }

      // extract the sample id
      std::string sample_id = req.substr (receive_pose_delta_pred_request.length(),
                                          req.length());
      
      // here we should check that and wait for it
      while (sm_ready_for_pose_delta_prediction == FALSE){
        // just wait
        taskDelay(ns2ticks(10000000000));
      }
      
      //  Send reply back to client and check if correct by composition
      std::string confirmation = confirm_prefix +
	receive_pose_delta_pred_request + std::to_string(global_sample_id);
      
      // init the reply message
      zmq::message_t reply (confirmation.length());
      // compy content into reply
      memcpy ((void *) reply.data (), confirmation.c_str(),
	      confirmation.length());
      // send confirmation
      socket.send (reply);
      std::cout << "      Confirmed: ready to receive pose delta prediction for ID: "<< global_sample_id << std::endl;
      
      // we receive the pose delta
      zmq::message_t pose_delta_msg;
      socket.recv (&pose_delta_msg, 0);
      std::string pose_delta = std::string(static_cast<char*>(pose_delta_msg.data()),
					   pose_delta_msg.size());
      
      // The prefix of the prediction
      std::string pose_pred_prefix = "pose_delta_prediction:";
      
      // make sure it is requested
      if (pose_delta.std::string::find(pose_pred_prefix) == std::string::npos){
        std::cout << "      POSE DELTA REQUEST FAILED!!!! ID: "<< global_sample_id
                  << std::endl;
        throw std::invalid_argument("see above");
      }
      
      //  Send reply back to client and check if correct by composition
      std::string confirmation_pose_pred = confirm_prefix +
	pose_pred_prefix + std::to_string(global_sample_id);
      
      // init the reply message
      zmq::message_t reply_pose_pred (confirmation_pose_pred.length());
      // compy content into reply
      memcpy ((void *) reply_pose_pred.data (), confirmation_pose_pred.c_str(),
	      confirmation_pose_pred.length());
      // send confirmation
      socket.send (reply_pose_pred);
      std::cout << "      Confirmed: reveived pose delta prediction for ID: "<< global_sample_id << std::endl;
      
      // convert string to float arrayValue
      
      // The prefix of the prediction
      std::string start_pose_delta_list_char = ":";
      int str_index;
      str_index = pose_delta.std::string::find(start_pose_delta_list_char);
      str_index = str_index + 2;
      std::string pose_delta_pred_string = pose_delta.substr(str_index, pose_delta.length()-str_index-1);
      pose_delta_pred_string.erase(std::remove(pose_delta_pred_string.begin(), pose_delta_pred_string.end(), '\n'),
				   pose_delta_pred_string.end());

      char delim = ','; // our delimiter

      std::istringstream ss(pose_delta_pred_string);
      std::string token;

      std::vector<std::string> pose_delta_string;
      while(std::getline(ss, token, delim)) {
        pose_delta_string.push_back(token);
      }

      int i;
      double diff;
      max_abs_diff_predicted_reference_state_pose_delta_x = 0.00;
      max_abs_diff_predicted_reference_state_pose_delta_q = 0.00;
      double diff_predicted_reference_state_pose_delta_x[N_CART+1];

      // position compensation
      for (i=1; i<=N_CART; ++i) {
        predicted_reference_state_pose_delta_x_inverse[i] = -std::atof(pose_delta_string[i-1].c_str());
        diff = std::abs(predicted_reference_state_pose_delta_x_inverse[i] +
			reference_state_pose_x[i] - reference_state_pose_x_base[i]);
	diff_predicted_reference_state_pose_delta_x[i] = diff;
        if(diff > max_abs_diff_predicted_reference_state_pose_delta_x){
          max_abs_diff_predicted_reference_state_pose_delta_x = diff;
        }
        //std::cout << "            position   " << i << " abs_diff: " << diff << "     abs(abs(" << predicted_reference_state_pose_delta_x_inverse[i] << ") - abs(" << reference_state_pose_delta_x_table[current_state_pose_delta][i] << "))" << std::endl;
      }

      // orientation compensation
      predicted_reference_state_pose_delta_q_inverse[_Q0_] = std::atof(pose_delta_string[N_CART+3].c_str());
      predicted_reference_state_pose_delta_q_inverse[_Q1_] = -std::atof(pose_delta_string[N_CART].c_str());
      predicted_reference_state_pose_delta_q_inverse[_Q2_] = -std::atof(pose_delta_string[N_CART+1].c_str());
      predicted_reference_state_pose_delta_q_inverse[_Q3_] = -std::atof(pose_delta_string[N_CART+2].c_str());

      // ensure unit length
      double q_norm;
      q_norm = sqrt(vec_mult_inner_size(predicted_reference_state_pose_delta_q_inverse,predicted_reference_state_pose_delta_q_inverse,N_QUAT));
      for (i=1; i<=N_QUAT; ++i) {
	predicted_reference_state_pose_delta_q_inverse[i] /= q_norm;
      }

      double q_diff[N_QUAT+1];
      quatRelative(reference_state_pose_q_base,reference_state_pose_q,q_diff);
      quatMult(q_diff,predicted_reference_state_pose_delta_q_inverse,q_diff);

      // norm of vector component of q_diff
      double q_diff_norm = sqrt(vec_mult_inner_size(&(q_diff[_Q0_]),&(q_diff[_Q0_]),N_CART));
      double so3_q_diff[N_CART+1];
      for (int i=1; i<=N_CART; ++i) {
	so3_q_diff[i] = 2.* acos(q_diff[_Q0_]) * q_diff[_Q0_+i] / (q_diff_norm + 1.e-10);
      }

      printf("\n----------------------------------------------------------------------------------\n");
      printf("Total pos error [m]: %f\n",sqrt(sqr(diff_predicted_reference_state_pose_delta_x[_X_])+
					      sqr(diff_predicted_reference_state_pose_delta_x[_Y_])+
					      sqr(diff_predicted_reference_state_pose_delta_x[_Z_])));
      printf("Pos [m] error      : %f %f %f\n",diff_predicted_reference_state_pose_delta_x[_X_],
	     diff_predicted_reference_state_pose_delta_x[_Y_],
	     diff_predicted_reference_state_pose_delta_x[_Z_]);

      printf("Angle err [deg]    : %f\n",acos(q_diff[_Q0_])*2/PI*180.);
      printf("SO3 error          : %f (%f)  %f (%f)  %f (%f)\n",so3_q_diff[_X_],so3_q_diff[_X_]/PI*180.,so3_q_diff[_Y_],so3_q_diff[_Y_]/PI*180.,so3_q_diff[_Z_],so3_q_diff[_Z_]/PI*180.);
      printf("----------------------------------------------------------------------------------\n\n");
      max_abs_diff_predicted_reference_state_pose_delta_q = acos(q_diff[_Q0_])*2/PI*180.0;

      //      std::cout << "Maximum absolute deviation between prediction and groundtruth: x=" <<
      //	max_abs_diff_predicted_reference_state_pose_delta_x <<
      //" alpha=" << max_abs_diff_predicted_reference_state_pose_delta_q  << std::endl;
      
      if(max_abs_diff_predicted_reference_state_pose_delta_x > max_allowed_abs_diff_predicted_reference_state_pose_delta_x ||
	 max_abs_diff_predicted_reference_state_pose_delta_q > max_allowed_abs_diff_predicted_reference_state_pose_delta_q){
	
        std::cout << "\n\n ERROR: The maximum allowd deviation x=" <<
	  max_allowed_abs_diff_predicted_reference_state_pose_delta_x << " or alpha=" <<
	  max_allowed_abs_diff_predicted_reference_state_pose_delta_q << " of the prediction was exceeded." << std::endl;
	std::cout << "\nYou have 10 seconds to run cc command to execute correction" << std::endl;
	
        continue_pose_correction = FALSE;

	int count = task_servo_calls;
        while (!continue_pose_correction) {
	  sleep(1);
	  if (task_servo_calls - count > 10000)
	    break;
        }

      } else {

	continue_pose_correction = TRUE;

      }

      // execute the pose correction
      if (continue_pose_correction){
	
	// position compensation
	for (i=1; i<=N_CART; ++i) {
	  reference_state_pose_x[i] = reference_state_pose_x[i] + predicted_reference_state_pose_delta_x_inverse[i];
	}
	
	// orientation compensation
	quatMult(reference_state_pose_q, predicted_reference_state_pose_delta_q_inverse,
		 reference_state_pose_q);
	
      }

      sm_ready_for_pose_delta_prediction = FALSE;
	

    }
  }

  return NULL;

}

/*!*****************************************************************************
*******************************************************************************
\note  add_joints_state
\date  Mai 2019

\remarks

write the Joints State to the json

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
add_joints_state(Json::Value &state)
{
  Json::Value th(Json::arrayValue);
  Json::Value th_dot(Json::arrayValue);
  Json::Value th_dot_dot(Json::arrayValue);
  Json::Value ufb(Json::arrayValue);
  Json::Value u(Json::arrayValue);
  Json::Value load(Json::arrayValue);

  // the following variables need to be assigned
  for (int i=1; i<=N_DOFS; ++i) {
	th.append(Json::Value(joint_state[i].th));
	th_dot.append(Json::Value(joint_state[i].thd));
	th_dot_dot.append(Json::Value(joint_state[i].thdd));
	ufb.append(Json::Value(joint_state[i].ufb));
	u.append(Json::Value(joint_state[i].u));
	load.append(Json::Value(joint_state[i].load));
  }
  state["joints_state"]["load"] = load;
  state["joints_state"]["u"] = u;
  state["joints_state"]["ufb"] = ufb;
  state["joints_state"]["th_d"] = th_dot;
  state["joints_state"]["th_d_d"] = th_dot_dot;
  state["joints_state"]["th"] = th;
}

/*!*****************************************************************************
*******************************************************************************
\note  add_endeffector_cart
\date  Mai 2019

\remarks

write the cart endeffector state to the json, for single endeffector only

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
add_endeffector_cart(Json::Value &state)
{
  Json::Value pos(Json::arrayValue);
  Json::Value vel(Json::arrayValue);
  Json::Value acc(Json::arrayValue);

  // the following variables need to be assigned
  for (int i=1; i<=N_CART; ++i) {
	pos.append(Json::Value(cart_state[HAND].x[i]));
	vel.append(Json::Value(cart_state[HAND].xd[i]));
	acc.append(Json::Value(cart_state[HAND].xdd[i]));
  }

  state["endeff_cartesian"]["acc"] = acc;
  state["endeff_cartesian"]["vel"] = vel;
  state["endeff_cartesian"]["position[x,y,z]"] = pos;
}

/*!*****************************************************************************
*******************************************************************************
\note  add_endeffector_orient
\date  Mai 2019

\remarks

write the orientation of endeffector to the json, for single endeffector only

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
add_endeffector_orient(Json::Value &state)
{
  Json::Value pos(Json::arrayValue);
  Json::Value vel(Json::arrayValue);
  Json::Value acc(Json::arrayValue);
  Json::Value ang_vel(Json::arrayValue);
  Json::Value ang_acc(Json::arrayValue);

  // the following variables need to be assigned
  for (int i=1; i<=N_QUAT; ++i) {
	pos.append(Json::Value(cart_orient[HAND].q[i]));
	vel.append(Json::Value(cart_orient[HAND].qd[i]));
	acc.append(Json::Value(cart_orient[HAND].qdd[i]));
  }

  for (int i=1; i<=N_CART; ++i) {
  	ang_vel.append(Json::Value(cart_orient[HAND].ad[i]));
	ang_acc.append(Json::Value(cart_orient[HAND].add[i]));
  }

  state["endeff_quaternion"]["ang_acc"] = ang_acc;
  state["endeff_quaternion"]["ang_vel[alpha,beta,gamma]"] = ang_vel;
  state["endeff_quaternion"]["acc"] = acc;
  state["endeff_quaternion"]["vel"] = vel;
  state["endeff_quaternion"]["position[q0,q1,q2,q3]"] = pos;
}

/*!*****************************************************************************
*******************************************************************************
\note  add_endeffector_calculated_force_torque
\date  june 2019

\remarks

write the calculated force and torque at the endeffector to the json,
for single endeffector only

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
add_endeffector_calculated_force_torque(Json::Value &state)
{
  Json::Value calc_force(Json::arrayValue);
  Json::Value calc_torque(Json::arrayValue);

  // the following variables need to be assigned
  for (int i=1; i<=N_CART; ++i) {
  	calc_force.append(Json::Value(misc_sensor[C_FX-1+i]));
	calc_torque.append(Json::Value(misc_sensor[C_MX-1+i]));
  }

  state["endeff_calc_force_torque"]["force[x,y,z]"] = calc_force;
  state["endeff_calc_force_torque"]["torque[x,y,z]"      ] = calc_torque;
}

/*!*****************************************************************************
*******************************************************************************
\note  add_experiment_data
\date  Mai 2019

\remarks

This function is used to add all other variables and infos we might needed
to log during our experiment.

*******************************************************************************
Function Parameters: [in]=input,[out]=output

none

******************************************************************************/
void
add_experiment_data(Json::Value &state)
{
  Json::Value pos_x_truth(Json::arrayValue);
  Json::Value pos_x_delta(Json::arrayValue);
  Json::Value pos_x_target(Json::arrayValue);

  Json::Value pos_q_truth(Json::arrayValue);
  Json::Value pos_q_delta(Json::arrayValue);
  Json::Value pos_q_target(Json::arrayValue);

  double reference_state_pose_q_base_inverse[N_QUAT+1];
  double q_tilted_inverse[N_QUAT+1];    // the special tilt approach for insertion
  double q_mis_grasp[N_QUAT+1]={0, 0.9990, -0.0436, 0, 0}; // in robot base coordinates

  // add the tracking error to the pose_delta to make image and robot state
  // maximaly consistent

  // translation offset is easy as addition is commutative
  vec_sub_size(reference_state_pose_x,reference_state_pose_x_base,N_CART,reference_state_pose_delta_x_adjusted);
  vec_add_size(reference_state_pose_delta_x_adjusted,
	       pos_error_vector,
	       N_CART,
	       reference_state_pose_delta_x_adjusted);
  // hack for bad grasp data collection
  printf("bad grasp simulation is running -- remove\n");
  //  reference_state_pose_delta_x_adjusted[_Y_] -= 0.005; // 5mm mis-grasped in y direction

  // orientation needs rotation matrix rules: q_adj = q_tilt_inv * q_current * q_base_inv
  quatRelative(reference_state_pose_q,ctarget_orient[HAND].q,q_tilted_inverse);
  for (int i=1; i<=N_QUAT; ++i) {

    if (i != _Q0_)
      q_tilted_inverse[i] *= -1.;

    reference_state_pose_q_base_inverse[i] = reference_state_pose_q_base[i];
    if (i != _Q0_)
      reference_state_pose_q_base_inverse[i] *= -1.;
    
  }
  
  quatMult(reference_state_pose_q_base_inverse,ctarget_orient[HAND].q,reference_state_pose_delta_q_adjusted);  
  quatMult(reference_state_pose_delta_q_adjusted,q_tilted_inverse,reference_state_pose_delta_q_adjusted);

  // hack for orientation mis-grasp
  quatMult(reference_state_pose_delta_q_adjusted,q_mis_grasp,reference_state_pose_delta_q_adjusted);

  //print_vec_size("delta_x_adj",reference_state_pose_delta_x_adjusted,N_CART);
  //print_vec_size("delta_x",reference_state_pose_delta_x_table[current_state_pose_delta],N_CART);  
  //print_vec_size("delta_q_adj",reference_state_pose_delta_q_adjusted,N_QUAT);
  //print_vec_size("delta_q",reference_state_pose_delta_q_table[current_state_pose_delta],N_QUAT);    

  // the following variables need to be assigned
  for (int i=1; i<=N_CART; ++i) {
    pos_x_truth.append(Json::Value(reference_state_pose_x_base[i]));
    //    pos_x_delta.append(Json::Value(reference_state_pose_delta_x_table[current_state_pose_delta][i]));
    pos_x_delta.append(Json::Value(reference_state_pose_delta_x_adjusted[i]));    
    pos_x_target.append(Json::Value(reference_state_pose_x[i]));
  }

  // the following variables need to be assigned
  for (int i=1; i<=N_QUAT; ++i) {
    pos_q_truth.append(Json::Value(reference_state_pose_q_base[i]));
    //    pos_q_delta.append(Json::Value(reference_state_pose_delta_q_table[current_state_pose_delta][i]));
    pos_q_delta.append(Json::Value(reference_state_pose_delta_q_adjusted[i]));    
    pos_q_target.append(Json::Value(reference_state_pose_q[i]));
  }

  state["experiment_data"]["socket_pose_truth"]["x"]  = pos_x_truth;
  state["experiment_data"]["socket_pose_delta"]["x"]  = pos_x_delta;
  state["experiment_data"]["socket_pose_target"]["x"] = pos_x_target;

  state["experiment_data"]["socket_pose_truth"]["q"]  = pos_q_truth;
  state["experiment_data"]["socket_pose_delta"]["q"]  = pos_q_delta;
  state["experiment_data"]["socket_pose_target"]["q"] = pos_q_target;
}
