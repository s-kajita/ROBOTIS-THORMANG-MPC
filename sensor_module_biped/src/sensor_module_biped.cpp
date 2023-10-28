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
  : control_cycle_sec_(0.01)
{
  module_name_ = "sensor_module_biped"; // set unique module name

  result_["test_sensor"] = 0.0;
}

SensorModuleBiped::~SensorModuleBiped()
{
  queue_thread_.join();
}

void SensorModuleBiped::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec/1000.0;
  queue_thread_ = boost::thread(boost::bind(&SensorModuleBiped::queueThread, this));
  
  fprintf(stderr, "sensor_module_biped:initialize()\n");
}

void SensorModuleBiped::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  //sub1_ = ros_node.subscribe("/sensor_module", 10, &SensorModuleBiped::topicCallback, this);

  /* publisher */
  pub1_rfoot = ros_node.advertise<std_msgs::UInt16MultiArray>("/rfoot_forces", 1, true);

  ros::WallDuration duration(control_cycle_sec_);
  //while(ros_node.ok())
  //  callback_queue.callAvailable(duration);
}

void SensorModuleBiped::topicCallback(const std_msgs::Int16::ConstPtr &msg)
{
	std::cout << "msg: " <<  msg->data << std::endl;
}

void SensorModuleBiped::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
    std::map<std::string, robotis_framework::Sensor *> sensors)
{
  /* right foot force sensors   (pitch ankle joint) */
  uint16_t ext_port_data_0 = dxls["joint4"]->dxl_state_->bulk_read_table_["external_port_data_1"];
  uint16_t ext_port_data_1 = dxls["joint4"]->dxl_state_->bulk_read_table_["external_port_data_2"];
  uint16_t ext_port_data_2 = dxls["joint5"]->dxl_state_->bulk_read_table_["external_port_data_1"];
  uint16_t ext_port_data_3 = dxls["joint5"]->dxl_state_->bulk_read_table_["external_port_data_2"];

	std_msgs::UInt16MultiArray msg;
	msg.data.resize(4);
	msg.data[0] = ext_port_data_0;
  msg.data[1] = ext_port_data_1;
  msg.data[2] = ext_port_data_2;
  msg.data[3] = ext_port_data_3;

  pub1_rfoot.publish(msg);

  result_["test_sensor"] = 0.0;
}

