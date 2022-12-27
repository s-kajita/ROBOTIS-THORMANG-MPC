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
#include "motion_module_lleg/motion_module_lleg.h"

using namespace thormang3;

MotionModuleLleg::MotionModuleLleg()
  : control_cycle_sec_(0.01),
  is_moving_(false)
{
  enable_       = false;
  module_name_  = "motion_module_lleg"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

	iTime = 0;
	Time = 0.0;
	
	JointNameList.clear();
	JointNameList.push_back("joint6");
	JointNameList.push_back("joint7");
	JointNameList.push_back("joint8");
	JointNameList.push_back("joint9");
	JointNameList.push_back("joint10");	
	JointNameList.push_back("joint11");
		
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

MotionModuleLleg::~MotionModuleLleg()
{
  queue_thread_.join();
}

void MotionModuleLleg::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	iTime = 0;
	Time = 0.0;
	
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&MotionModuleLleg::queueThread, this));

  fprintf(stderr, "%s:initialize()\n",module_name_.c_str());
}

void MotionModuleLleg::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  sub1_ = ros_node.subscribe("/lleg_cmd", 10, &MotionModuleLleg::topicCallback, this);

  /* publisher */
 // pub1_ = ros_node.advertise<std_msgs::Float32>("/sample_motion", 1, true);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/* -------------------- Callback ---------------------------- */

void MotionModuleLleg::topicCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
//  std_msgs::Float32MultiArray std_msg;
//  std_msg.data = msg->data;
//  pub1_.publish(std_msg);

	start_time = Time;
	start_pose = goal_pose;
  
  fprintf(stderr, "left leg goal_pose = [");
  for(int i = 0; i < goal_pose.size(); i++){
    goal_pose(i) = (double)msg->data[i];
    fprintf(stderr, " %g ",goal_pose(i));
    if( i != goal_pose.size() -1 ){
    	fprintf(stderr, ",");
    }
    else{
    	fprintf(stderr, "]\n");
    }	
  }
  
  T_interp = 2.0;
  s_interp = 0.0;
}



/* =================================== MAIN PROCESS ===================================== */ 

void MotionModuleLleg::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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
    
		for(int i = 0; i < start_pose.size(); i++){
		  fprintf(stderr, "left leg start_pose(%d)=%g\n",i,start_pose(i));
		}
    
    goal_pose = start_pose;
    s_interp = 1.0;
 
 		for(int i = 0; i < goal_pose.size(); i++){
		  fprintf(stderr, "left leg goal_pose(%d)=%g\n",i,goal_pose(i));
		}

    
    firsttime = false;
    fprintf(stderr, "motion_module_lleg: enable\n");    
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

void MotionModuleLleg::stop()
{
  return;
}

bool MotionModuleLleg::isRunning()
{
  return false;
}

