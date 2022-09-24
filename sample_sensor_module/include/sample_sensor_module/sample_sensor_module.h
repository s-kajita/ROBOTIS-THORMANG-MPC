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
 *  Created on: 2016. 3. 22.
 *      Author: Jay Song
 */

#ifndef SAMPLE_SENSOR_MODULE_H_
#define SAMPLE_SENSOR_MODULE_H_

#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math.h"


namespace thormang3
{

class SampleSensor : public robotis_framework::SensorModule, public robotis_framework::Singleton<SampleSensor>
{
public:
  SampleSensor();
  ~SampleSensor();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  void queueThread();

  int             control_cycle_msec_;

  boost::thread   queue_thread_;
  boost::mutex    publish_mutex_;

  ros::Publisher  sample_status_pub_;

  bool firsttime;
};


}


#endif /* SAMPLE_SENSOR_MODULE_H_ */
