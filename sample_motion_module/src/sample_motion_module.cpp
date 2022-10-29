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
  : control_cycle_sec_(0.01),
  is_moving_(false)
{
  enable_       = false;
  module_name_  = "sample_motion_module"; // set unique module name
  control_mode_ = robotis_framework::PositionControl;

  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();
  result_["joint5"] = new robotis_framework::DynamixelState();

  NumberOfJoint = 3;

  goal_joint_position_      = Eigen::VectorXd::Zero(NumberOfJoint);
  start_joint_position_     = Eigen::VectorXd::Zero(NumberOfJoint);
  
  firsttime = true;
}

SampleMotionModule::~SampleMotionModule()
{
  queue_thread_.join();
}

void SampleMotionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
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
  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/joint_pose_msg", 5,
                                                          &SampleMotionModule::jointPoseMsgCallback, this);

  /* publisher */
 // pub1_ = ros_node.advertise<std_msgs::Float32>("/sample_motion", 1, true);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/* -------------------- Callback ---------------------------- */

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

void SampleMotionModule::jointPoseMsgCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  goal_joint_pose_msg_ = *msg;

  if (is_moving_ == false)
  {
    traj_generate_tread_ = new boost::thread(boost::bind(&SampleMotionModule::jointTrajGenerateProc, this));
    delete traj_generate_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

void SampleMotionModule::jointTrajGenerateProc()
{
#if 0
  if (goal_joint_pose_msg_.time <= 0.0)
  {
    /* set movement time */
    double tol        = 10 * DEGREE2RADIAN; // rad per sec
    double mov_time   = 2.0;

    int    id    = joint_name_to_id_[goal_joint_pose_msg_.name];

    double ini_value  = goal_joint_position_(id);
    double tar_value  = goal_joint_pose_msg_.value;
    double diff       = fabs(tar_value - ini_value);

    mov_time_ = diff / tol;
    int _all_time_steps = int(floor((mov_time_ / control_cycle_sec_) + 1.0));
    mov_time_ = double(_all_time_steps - 1) * control_cycle_sec_;

    if (mov_time_ < mov_time)
      mov_time_ = mov_time;
  }
  else
  {
    mov_time_ = goal_joint_pose_msg_.time;
  }
#endif

  mov_time_ = 3.0;  /* s */  

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;

  goal_joint_tra_.resize(all_time_steps_, NumberOfJoint);


  /* calculate joint trajectory */
  for (int j = 0; j < goal_joint_position_.size(); j++)
  {
    double ini_value = goal_joint_position_(j);
    double tar_value = (double)goal_joint_pose_msg_.data[j];

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                control_cycle_sec_,
                                                                mov_time_);

    goal_joint_tra_.block(0, j, all_time_steps_, 1) = tra;
  }

  cnt_        = 0;
  is_moving_  = true;

  ROS_INFO("[start] send trajectory");
}

/* =================================== MAIN PROCESS ===================================== */ 

void SampleMotionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                   std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  if (firsttime ){
    // ----------  set goal_joint_position_ as the initial pose  ------------- 
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

      goal_joint_position_(j) = dxl->dxl_state_->present_position_;
      j++;
    } 
    
    firsttime = false;
    fprintf(stderr, "sample_motion_module: enable\n");    
  }


  // ...

  result_["joint3"]->goal_position_ = goal_joint_position_(0);
  result_["joint4"]->goal_position_ = goal_joint_position_(1);
  result_["joint5"]->goal_position_ = goal_joint_position_(2);

}

void SampleMotionModule::stop()
{
  return;
}

bool SampleMotionModule::isRunning()
{
  return false;
}

