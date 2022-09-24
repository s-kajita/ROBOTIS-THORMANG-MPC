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

/*
 * sample_sensor_module.h
 *
 *  Created on: 2020. 12. 25
 *      Author: Jay Song
 */


#include "sample_sensor_module/sample_sensor_module.h"

using namespace thormang3;

SampleSensor::SampleSensor()
: control_cycle_msec_(8)
{
  module_name_ = "sample_sensor_module"; // set unique module name
}

SampleSensor::~SampleSensor()
{
  queue_thread_.join();

  firsttime = true;
}

void SampleSensor::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&SampleSensor::queueThread, this));

  fprintf(stderr, "sample_sensor_module:initialize()\n");
}


void SampleSensor::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publisher */
  sample_status_pub_  = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/sample_sensor", 1);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void SampleSensor::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
			   std::map<std::string, robotis_framework::Sensor *> sensors)
{
  if (firsttime ){
    firsttime = false;
    fprintf(stderr, "sample_sensor_module: enable\n");
  }

}
