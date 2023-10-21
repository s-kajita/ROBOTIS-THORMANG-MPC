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
#include "sensor_module_biped/sensor_module_biped.h"

using namespace ROBOTIS;

SensorModuleBiped::SensorModuleBiped()
  : control_cycle_msec_(8)
{
  module_name_ = "test_sensor_module"; // set unique module name

  result_["test_sensor"] = 0.0;
}

SensorModuleBiped::~SensorModuleBiped()
{
  queue_thread_.join();
}

void SensorModuleBiped::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&SensorModuleBiped::queueThread, this));
}

void SensorModuleBiped::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  //sub1_ = ros_node.subscribe("/tutorial_topic", 10, &SensorModuleBiped::topicCallback, this);

  /* publisher */
  pub1_ = ros_node.advertise<std_msgs::UInt16>("/foot_force_sensor", 1, true);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

/*
void SensorModuleBiped::topicCallback(const std_msgs::Int16::ConstPtr &msg)
{
  std_msgs::Int16 msg_int16;
  msg_int16.data = msg->data;
  pub1_.publish(msg_int16);
}
*/

void SensorModuleBiped::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
    std::map<std::string, robotis_framework::Sensor *> sensors)
{
  uint16_t ext_port_data_1 = dxls["joint4"]->dxl_state_->bulk_read_table_["external_port_data_1"];
  uint16_t ext_port_data_2 = dxls["joint4"]->dxl_state_->bulk_read_table_["external_port_data_2"];

	std_msgs::UInt16 msg;
	msg.data = ext_port_data_1;
  pub1_.publish(msg);

  result_["test_sensor"] = 0.0;
}

