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

	iTime = 0;
	Time = 0.0;
	
	
	JointNameList.clear();
	JointNameList.push_back("joint0");
	JointNameList.push_back("joint1");
	JointNameList.push_back("joint2");
	JointNameList.push_back("joint3");
	JointNameList.push_back("joint4");
	JointNameList.push_back("joint5");
				
	for( auto jname : JointNameList ){
		result_[jname] = new robotis_framework::DynamixelState();
	}

  goal_pose  = Eigen::VectorXd::Zero(JointNameList.size());
  start_pose = Eigen::VectorXd::Zero(JointNameList.size());

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
	iTime = 0;
	Time = 0.0;
	
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
  sub_poseData = ros_node.subscribe("/rleg_cmd",  10, &MotionModuleRleg::poseData_callback, this);
  sub_poseName = ros_node.subscribe("/rleg_name", 10, &MotionModuleRleg::poseName_callback, this);
  
  /* publisher */
 // pub1_ = ros_node.advertise<std_msgs::Float32>("/sample_motion", 1, true);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/* -------------------- Callback ---------------------------- */

void MotionModuleRleg::poseData_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	start_time = Time;
	start_pose = goal_pose;
  
  fprintf(stderr, "right leg goal_pose = [");
  for(int i = 0; i < goal_pose.size(); i++){
    goal_pose(i) = (double)msg->data[i];
    fprintf(stderr, " %g ", goal_pose(i));
    if( i != goal_pose.size()-1){
    	fprintf(stderr, ",");
    }
    else {
    	fprintf(stderr, "]\n");
    }
  }
  
  T_interp = 2.0;
  s_interp = 0.0;
}

void MotionModuleRleg::poseName_callback(const std_msgs::String::ConstPtr &msg)
{

	fprintf(stderr, "pose name = %s \n",msg->data.c_str());

#if 0
	start_time = Time;
	start_pose = goal_pose;
  
  fprintf(stderr, "right leg goal_pose = [");
  for(int i = 0; i < goal_pose.size(); i++){
    goal_pose(i) = (double)msg->data[i];
    fprintf(stderr, " %g ", goal_pose(i));
    if( i != goal_pose.size()-1){
    	fprintf(stderr, ",");
    }
    else {
    	fprintf(stderr, "]\n");
    }
  }
  
  T_interp = 2.0;
  s_interp = 0.0;
#endif
}



/* =================================== MAIN PROCESS ===================================== */ 

void MotionModuleRleg::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

	Time = iTime * control_cycle_sec_;
	iTime++;

  if (firsttime ){
    // ----------  set goal_pose as the initial pose  ------------- 
		for( int j=0; j < JointNameList.size(); j++){
			std::string joint_name = JointNameList[j];
			
      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;     // second field = dynamixel pointer
      else
        continue;

      start_pose(j) = dxl->dxl_state_->present_position_;		
		}
    
    goal_pose = start_pose;
    s_interp = 1.0;
    
    firsttime = false;
    fprintf(stderr, "motion_module_rleg: enable\n");    
  }


  // ...
  if( Time < start_time){
  	s_interp = 0.0;
  }
  else if( Time < start_time+T_interp ){
  	s_interp = (Time-start_time)/T_interp;
  }
  else {
  	s_interp = 1.0;
  }
  	
  pose = (1.0-s_interp)*start_pose + s_interp*goal_pose;

	for( int j=0; j < JointNameList.size(); j++){
		std::string jname = JointNameList[j];
		result_[jname]->goal_position_ = pose(j);
	}
		
}

void MotionModuleRleg::stop()
{
  return;
}

bool MotionModuleRleg::isRunning()
{
  return false;
}

