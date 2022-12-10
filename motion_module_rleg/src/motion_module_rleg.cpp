/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <stdio.h>
#include "motion_module_rleg/motion_module_rleg.h"

using namespace thormang3;

MotionModuleRleg::MotionModuleRleg()
  : control_cycle_sec_(0.01),
  is_moving_(false)
{
  enable_       = false;
  module_name_  = "motion_module_rleg"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

	int_time = 0;
	dbl_time = 0.0;
	
	result_["joint7"] = new robotis_framework::DynamixelState();
  result_["joint8"] = new robotis_framework::DynamixelState();

  NumberOfJoint = 2;

  goal_pose  = Eigen::VectorXd::Zero(NumberOfJoint);
  start_pose = Eigen::VectorXd::Zero(NumberOfJoint);

	start_time = 0.0;
  T_interp   = 1.0;
  s_interp   = 0.0;
  
  firsttime = true;
}

MotionModuleRleg::~MotionModuleRleg()
{
  queue_thread_.join();
}

void MotionModuleRleg::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	int_time = 0;
	dbl_time = 0.0;
	
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&MotionModuleRleg::queueThread, this));

  fprintf(stderr, "motion_module_rleg:initialize()\n");
}

void MotionModuleRleg::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  sub1_ = ros_node.subscribe("/rleg_cmd", 10, &MotionModuleRleg::topicCallback, this);

  /* publisher */
 // pub1_ = ros_node.advertise<std_msgs::Float32>("/sample_motion", 1, true);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/* -------------------- Callback ---------------------------- */

void MotionModuleRleg::topicCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
//  std_msgs::Float32MultiArray std_msg;
//  std_msg.data = msg->data;
//  pub1_.publish(std_msg);

	start_time = dbl_time;
	start_pose = goal_pose;
  
  for(int i = 0; i < goal_pose.size(); i++){
    goal_pose(i) = (double)msg->data[i];
    fprintf(stderr, "goal_pose(%d)=%g\n",i,goal_pose(i));
  }
  
  T_interp = 2.0;
  s_interp = 0.0;
}



/* =================================== MAIN PROCESS ===================================== */ 

void MotionModuleRleg::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

	dbl_time = int_time * control_cycle_sec_;
	int_time++;

  if (firsttime ){
    // ----------  set goal_pose as the initial pose  ------------- 
    int j=0;
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
         state_iter != result_.end(); 
         state_iter++)
    {
    	std::string joint_name = state_iter->first;  // first field = joint name
    
      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;     // second field = dynamixel pointer
      else
        continue;

      start_pose(j) = dxl->dxl_state_->present_position_;
      j++;
    } 
    fprintf(stderr, "number of active joints = %d\n",j);
    
    goal_pose = start_pose;
    s_interp = 1.0;
    
    firsttime = false;
    fprintf(stderr, "motion_module_rleg: enable\n");    
  }


  // ...
  if( dbl_time < start_time){
  	s_interp = 0.0;
  }
  else if( dbl_time < start_time+T_interp ){
  	s_interp = (dbl_time-start_time)/T_interp;
  }
  else {
  	s_interp = 1.0;
  }
  	
  pose = (1.0-s_interp)*start_pose + s_interp*goal_pose;

  result_["joint7"]->goal_position_ = pose(0);
  result_["joint8"]->goal_position_ = pose(1);

}

void MotionModuleRleg::stop()
{
  return;
}

bool MotionModuleRleg::isRunning()
{
  return false;
}

