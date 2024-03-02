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

#ifndef MOTION_MODULE_BIPED_H_
#define MOTION_MODULE_BIPED_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

namespace thormang3
{

class MotionModuleBiped
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<MotionModuleBiped>
{
public:
  MotionModuleBiped();
  virtual ~MotionModuleBiped();

  /* ROS Topic Callback Functions */
  void cmdData_callback(const std_msgs::Float32MultiArray::ConstPtr &msg);		// topic /biped_cmd
	void poseName_callback(const std_msgs::String::ConstPtr &msg);							// topic /biped_name

  /* ROS Calculation Functions */
  void jointTrajGenerateProc();  

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  
  
private:
	int			iTime;
	double 	Time;
  double  control_cycle_sec_;
  boost::thread queue_thread_;
  boost::thread  *traj_generate_tread_;
 
  /* sample subscriber & publisher */
  ros::Subscriber sub_cmdData;
  ros::Subscriber	sub_poseName;
  //ros::Publisher  pub1_;

  void queueThread();

	std::vector<std::string> JointNameList;
	
	std::vector<std::string> PoseNameList;				// topic /biped_name  command name
	std::vector<Eigen::VectorXd> PoseList;		// topic /biped_name  pose data
	Eigen::VectorXd pose_offset;
	Eigen::VectorXd joint_dir;			  // joint direction  +1 or -1

  Eigen::VectorXd goal_pose;
  Eigen::VectorXd start_pose;
  Eigen::VectorXd pose;
  double start_time;
  double T_interp;
	double s_interp;

  bool firsttime;
  
  /* trajectory */
  bool    is_moving_;
  double  mov_time_;
  int     cnt_;
  int     all_time_steps_;

  

};

}

#endif /* MOTION_MODULE_BIPED_H_ */
