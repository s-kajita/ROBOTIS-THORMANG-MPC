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
#include "sample_motion_module/sample_motion_module.h"

using namespace thormang3;

SampleMotionModule::SampleMotionModule()
  : control_cycle_msec_(8)
{
  enable_       = false;
  module_name_  = "sample_motion_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();
  result_["joint5"] = new robotis_framework::DynamixelState();

  goal_joint_position_      = Eigen::VectorXd::Zero(3);
  
  firsttime = true;
}

SampleMotionModule::~SampleMotionModule()
{
  queue_thread_.join();
}

void SampleMotionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&SampleMotionModule::queueThread, this));

  fprintf(stderr, "sample_motion_module:initialize()\n");
}

void SampleMotionModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  sub1_ = ros_node.subscribe("/sample_cmd", 10, &SampleMotionModule::topicCallback, this);

  /* publisher */
 // pub1_ = ros_node.advertise<std_msgs::Float32>("/sample_motion", 1, true);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void SampleMotionModule::topicCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
//  std_msgs::Float32MultiArray std_msg;
//  std_msg.data = msg->data;
//  pub1_.publish(std_msg);
  
  for(int i = 0; i < goal_joint_position_.size(); i++){
    goal_joint_position_(i) = (double)msg->data[i];
    fprintf(stderr, "goal_joint_position_(%d)=%g\n",i,goal_joint_position_(i));
  }
}


void SampleMotionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  if (firsttime ){
    firsttime = false;
    fprintf(stderr, "sample_motion_module: enable\n");
  }
  
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin(); state_iter != result_.end();
      state_iter++)
  {
    int32_t p_pos = dxls[state_iter->first]->dxl_state_->present_position_;
    int32_t g_pos = dxls[state_iter->first]->dxl_state_->goal_position_;
  }

  // ...

  result_["joint3"]->goal_position_ = goal_joint_position_(0);
  result_["joint4"]->goal_position_ = goal_joint_position_(1);
  result_["joint5"]->goal_position_ = goal_joint_position_(2);

  //result_["joint1"]->present_position_ = 0;
  //result_["joint2"]->present_position_ = 1;
}

void SampleMotionModule::stop()
{
  return;
}

bool SampleMotionModule::isRunning()
{
  return false;
}

