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

#ifndef SAMPLE_MOTION_MODULE_H_
#define SAMPLE_MOTION_MODULE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "robotis_framework_common/motion_module.h"

namespace thormang3
{

class SampleMotionModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<SampleMotionModule>
{
private:
  int           control_cycle_msec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Subscriber sub1_;
  ros::Publisher  pub1_;

  void queueThread();

  Eigen::VectorXd goal_joint_position_;

  bool firsttime;
  
public:
  SampleMotionModule();
  virtual ~SampleMotionModule();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
};

}

#endif /* SAMPLE_MOTION_MODULE_H_ */
