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

#ifndef SENSOR_MODULE_BIPED_H_
#define SENSOR_MODULE_BIPED_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/sensor_module.h"

/******** for MIP SDK  (IMU)  ******/
#include <mip/mip_all.hpp>	 // location mip_sdk/src/mip/mip_all.hpp
#include <array>
#include "../mip_sdk/examples/example_utils.hpp"

using namespace mip;

namespace ROBOTIS
{

class SensorModuleBiped
  : public robotis_framework::SensorModule,
    public robotis_framework::Singleton<SensorModuleBiped>
{
private:
  double           control_cycle_sec_;
  boost::thread queue_thread_;

  /* sample subscriber & publisher */
  ros::Subscriber sub1_;
  ros::Publisher  pub1_rfoot;
  ros::Publisher  pub1_gyro;

  void queueThread();

	/*=============== IMU sensor variables ================*/
	std::unique_ptr<ExampleUtils> utils;
  //std::unique_ptr<mip::DeviceInterface> imu_device;
	
	//Gyro bias
	float gyro_bias[3];
  float sensor_to_vehicle_rotation_euler[3];

  DispatchHandler sensor_data_handlers[3];
  DispatchHandler filter_data_handlers[5];
	
	//Device data stores
	data_sensor::GpsTimestamp sensor_gps_time;
	data_sensor::ScaledAccel  sensor_accel;
	data_sensor::ScaledGyro   sensor_gyro;

	data_filter::Timestamp    filter_gps_time;
	data_filter::Status       filter_status;
	data_filter::EulerAngles  filter_euler_angles;
	data_filter::CompAngularRate  filter_comp_angular_rate;
	data_filter::CompAccel    filter_comp_accel;

	bool filter_state_running = false;

public:
  SensorModuleBiped();
  virtual ~SensorModuleBiped();

  /* ROS Topic Callback Functions */
  void topicCallback(const std_msgs::Int16::ConstPtr &msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void setup_IMU();
  void read_IMU(std::vector<float> data);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);
};

}

#endif /* SENSOR_MODULE_BIPED_H_ */
