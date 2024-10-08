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
#include "motion_module_biped/motion_module_biped.h"

using namespace thormang3;

MotionModuleBiped::MotionModuleBiped()
  : control_cycle_sec_(0.01),
  is_moving_(false)
{
  enable_       = false;
  module_name_  = "motion_module_biped"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;
  firsttime = true;

	iTime = 0;
	Time = 0.0;
	
	
	JointNameList.clear();
	JointNameList.push_back("joint0");
	JointNameList.push_back("joint1");
	JointNameList.push_back("joint2");
	JointNameList.push_back("joint3");
	JointNameList.push_back("joint4");
	JointNameList.push_back("joint5");
	
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
	pose_offset= Eigen::VectorXd::Zero(JointNameList.size());
	joint_dir  = Eigen::VectorXd::Zero(JointNameList.size());
	
	joint_dir << 1.9048,-1,1,1,1,1,  1.9048,-1,-1,-1,-1,1;		// align joint rotation direction to XYZ frame (hip_yaw= 40T/21T)


	pose_offset << 4.509,0,0,-45.0,-5.0,0,  0,0,0,-45.0,-5.0,0;   // hip_yaw offset,  knee & ankle: structual offset [deg]
	pose_offset *= DEGREE2RADIAN;


	start_time = 0.0;
  T_interp   = 1.0;
  s_interp   = 0.0;

	//---------- set pose command and pose data ---------
	PoseNameList.clear(); PoseList.clear();
	Eigen::VectorXd pose(JointNameList.size()); 
	
	PoseNameList.push_back("initial");     pose << 0,0,0,0,0,0,        0,0,0,0,0,0;       PoseList.push_back(DEGREE2RADIAN*pose);
	PoseNameList.push_back("halfsitting"); pose << 0,0,-30,60,-30,0,   0,0,-30,60,-30,0;  PoseList.push_back(DEGREE2RADIAN*pose);  
	PoseNameList.push_back("squat");       pose << 0,0,-60,120,-60,0,  0,0,-60,120,-60,0; PoseList.push_back(DEGREE2RADIAN*pose);  
	PoseNameList.push_back("openhipyaw");  pose << -90,0,0,0,0,0,      90,0,0,0,0,0;      PoseList.push_back(DEGREE2RADIAN*pose); 
	PoseNameList.push_back("spreadlegs");  pose << 0,-20,0,0,0,20,     0,20,0,0,0,-20;    PoseList.push_back(DEGREE2RADIAN*pose); 
	
	wp_count = 0;
	playing  = false;
}

MotionModuleBiped::~MotionModuleBiped()
{
  queue_thread_.join();
}

void MotionModuleBiped::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	iTime = 0;
	Time = 0.0;
	
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&MotionModuleBiped::queueThread, this));

  fprintf(stderr, "motion_module_biped:initialize()\n");
  
  //----------- get walking pattern ----------
	WalkingPattern.clear();
  
  std::ifstream pattern_file("/home/kajita/pattern/step.csv");
  std::string line;
  
  int jsize = 0;
  while (getline(pattern_file, line)) {
    if(line[0]=='%' || line[0] == '#') continue;    
    	std::vector<std::string> strvec = split(line, ',');
      
      if (jsize == 0){
      	jsize = strvec.size();
        pose.resize(jsize); 
      }
      for (int i=0; i<jsize;i++){
      	//fprintf(stderr, "%g, ", stof(strvec.at(i)));
      	pose[i] = stof(strvec.at(i));
      }
      //fprintf(stderr, "\n");
      WalkingPattern.push_back(pose);
	}
	
	std::cout << "jsize = " << jsize << std::endl;
	std::cout << "WalkingPattern size= " << WalkingPattern.size() << std::endl;

	wp_count = 0;
	playing = false;
}

void MotionModuleBiped::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  sub_cmdData  = ros_node.subscribe("/biped_cmd",  10, &MotionModuleBiped::cmdData_callback, this);
  sub_poseName = ros_node.subscribe("/biped_pose", 10, &MotionModuleBiped::poseName_callback, this);
  sub_play     = ros_node.subscribe("/biped_play", 10, &MotionModuleBiped::play_callback, this);
   
  /* publisher */
 // pub1_ = ros_node.advertise<std_msgs::Float32>("/sample_motion", 1, true);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/* -------------------- Callback ---------------------------- */

void MotionModuleBiped::cmdData_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	//------------ Check message size ------------
  if(msg->data.size() != goal_pose.size()){
    fprintf(stderr, "[ERROR] data.size()= %d, whereas pose.size()= %d\n",(int)msg->data.size(),(int)goal_pose.size());
    return;
  } 
  
	start_time = Time;
	start_pose = goal_pose;  
  
  fprintf(stderr, "biped goal_pose = [");
  for(int i = 0; i < goal_pose.size(); i++){
    goal_pose(i) = (double)msg->data[i];
    fprintf(stderr, " %g ", goal_pose(i)*RADIAN2DEGREE);
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

void MotionModuleBiped::poseName_callback(const std_msgs::String::ConstPtr &msg)
{
	int i;
	
	for(i = 0; i < PoseNameList.size(); i++){
		if( msg->data == PoseNameList[i] ){
			fprintf(stderr, "pose name = %s \n",msg->data.c_str());
			std::cout << PoseList[i].transpose()*RADIAN2DEGREE << std::endl;
			
			start_time = Time;
			start_pose = goal_pose;
			goal_pose  = PoseList[i];
			
			T_interp = 2.0;
			s_interp = 0.0;
			break;
		}
	}
	if( i == PoseNameList.size() ){
		std::cout << msg->data << " doesn't exist." << std::endl;
	}
	
}

void MotionModuleBiped::play_callback(const std_msgs::String::ConstPtr &msg)
{
	fprintf(stderr, "play pattern: %s \n",msg->data.c_str());
	
	start_time = Time;
	start_pose = goal_pose;  
  
  goal_pose = WalkingPattern[0];
    
  T_interp = 2.0;
  s_interp = 0.0;
  
  playing = true;
  wp_count = 0;
}

/* =================================== MAIN PROCESS ===================================== */ 

void MotionModuleBiped::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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

      start_pose(j) = dxl->dxl_state_->present_position_/joint_dir(j) - pose_offset(j);		// Dynamixel's position = joint_dir*(start_pose + pose_offset)
		}
		
    std::cout << "start_pose = \n" << start_pose.transpose()*RADIAN2DEGREE << std::endl;
   
    goal_pose = start_pose;
    s_interp = 1.0;
    
    firsttime = false;
    playing   = false;
    fprintf(stderr, "motion_module_biped: enable\n");    
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
  
  if( playing && s_interp == 1.0){
  	pose = WalkingPattern[ wp_count ];
  	//std::cout << WalkingPattern[ wp_count ].transpose()*RADIAN2DEGREE << std::endl;
  	if( wp_count < WalkingPattern.size()-2 ){
  		wp_count++;
  	}
  	else {
  		playing = false;
  		goal_pose = pose;
  	}
  }

	for( int j=0; j < JointNameList.size(); j++){
		std::string jname = JointNameList[j];
		result_[jname]->goal_position_ = joint_dir(j) * (pose(j) + pose_offset(j));
	}
		
}

void MotionModuleBiped::stop()
{
  return;
}

bool MotionModuleBiped::isRunning()
{
  return false;
}

