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

#ifndef MOTION_MODULE_RLEG_H_
#define MOTION_MODULE_RLEG_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

namespace thormang3
{

class MotionModuleRleg
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<MotionModuleRleg>
{
public:
  MotionModuleRleg();
  virtual ~MotionModuleRleg();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

  /* ROS Calculation Functions */
  void jointTrajGenerateProc();  

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  
  
private:
	int			int_time;
	double 	dbl_time;
  double  control_cycle_sec_;
  boost::thread queue_thread_;
  boost::thread  *traj_generate_tread_;
 
  /* sample subscriber & publisher */
  ros::Subscriber sub1_;
  ros::Publisher  pub1_;

  std_msgs::Float32MultiArray goal_joint_pose_msg_;

  void queueThread();

  Eigen::VectorXd goal_pose;
  Eigen::VectorXd start_pose;
  Eigen::VectorXd pose;
  double start_time;
  double T_interp;
	double s_interp;

  bool firsttime;
  
  /* trajectory */
  int	  NumberOfJoint;
  bool    is_moving_;
  double  mov_time_;
  int     cnt_;
  int     all_time_steps_;

  

};

}

#endif /* MOTION_MODULE_RLEG_H_ */
