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
using namespace mip;

SensorModuleBiped::SensorModuleBiped()
  : control_cycle_sec_(0.01)
{
  module_name_ = "sensor_module_biped"; // set unique module name

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;
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
  ROS_INFO("Starting sensor_module_biped");
  
  setup_IMU();		// setup IMU Microstrain GX5-AHRS(GX5-25)
}


//========== setup IMU GX5-AHRS(GX5-25) =======

void SensorModuleBiped::setup_IMU()
{   
	utils  = move(openFromArgs("/dev/ttyACM0", "115200", ""));
  //imu_device = move(utils->device);
  std::unique_ptr<mip::DeviceInterface>& imu_device = utils->device;
  
  printf("ping the IMU:  ");
  if(commands_base::ping(*imu_device) == CmdResult::ACK_OK)
  	printf("OK\n");
  else
    printf("NG\n");
    
  printf("set IMU idele: ");
  if(commands_base::setIdle(*imu_device) == CmdResult::ACK_OK)
  	printf("OK\n");
  else
    printf("NG\n");
 
	printf("load default settings: ");
  if(commands_3dm::defaultDeviceSettings(*imu_device) == CmdResult::ACK_OK)
  	printf("OK\n");
  else
    printf("NG\n");

  //
  //Load the device default settings (so the device is in a known state)
  //

	gyro_bias[0]=0; gyro_bias[1]=0; gyro_bias[2]=0;
	
  const uint32_t sampling_time = 2000; // The default is 15000 ms and longer sample times are recommended 
  const int32_t old_mip_sdk_timeout = imu_device->baseReplyTimeout();
  printf("Capturing gyro bias. This will take %d seconds \n", sampling_time/1000);
  imu_device->setBaseReplyTimeout(sampling_time * 2);

  if(commands_3dm::captureGyroBias(*imu_device, sampling_time, gyro_bias) != CmdResult::ACK_OK)
    printf("ERROR: Could not capture gyro bias!\n");

  if(commands_3dm::saveGyroBias(*imu_device) != CmdResult::ACK_OK)
    printf("ERROR: Could not save gyro bias!\n");

  const uint8_t fn_selector = 1;
  const uint8_t device_selector = 3;
  const uint8_t enable_flag = 1;
  if(commands_3dm::writeDatastreamControl(*imu_device, device_selector, enable_flag) != CmdResult::ACK_OK)
    printf("ERROR: Could not enable device data stream!\n");

  // Reset the timeout
  imu_device->setBaseReplyTimeout(old_mip_sdk_timeout);

  printf("Gyro bias: %f %f %f with sampling time: %d ms.\n",  gyro_bias[0], gyro_bias[1], gyro_bias[2],sampling_time);

  //
  //Setup Sensor data format to 100 Hz
  //

  uint16_t sensor_base_rate;

  //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
  //We could have also set it directly with information from the datasheet.

  if(commands_3dm::imuGetBaseRate(*imu_device, &sensor_base_rate) != CmdResult::ACK_OK)
    printf("ERROR: Could not get sensor base rate format!\n");

  const uint16_t sensor_sample_rate = 100; // Hz
  const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;


  std::array<DescriptorRate, 3> sensor_descriptors = {{
    { data_sensor::DATA_TIME_STAMP_GPS, sensor_decimation },
    { data_sensor::DATA_ACCEL_SCALED,   sensor_decimation },
    { data_sensor::DATA_GYRO_SCALED,    sensor_decimation },
  }};

  if(commands_3dm::writeImuMessageFormat(*imu_device, sensor_descriptors.size(), sensor_descriptors.data()) != CmdResult::ACK_OK)
    printf("ERROR: Could not set sensor message format!\n");

  //
  //Setup FILTER data format
  //

  uint16_t filter_base_rate;

  if(commands_3dm::filterGetBaseRate(*imu_device, &filter_base_rate) == CmdResult::ACK_OK)
  	printf("filter_base_rate=%d\n",filter_base_rate);
  else
    printf("ERROR: Could not get filter base rate format!\n");

  const uint16_t filter_sample_rate = 500; // Hz
  const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

  std::array<DescriptorRate, 5> filter_descriptors = {{
    { data_filter::DATA_FILTER_TIMESTAMP, filter_decimation },
    { data_filter::DATA_FILTER_STATUS,    filter_decimation },
    { data_filter::DATA_ATT_EULER_ANGLES, filter_decimation },
    { data_filter::DATA_COMPENSATED_ANGULAR_RATE, filter_decimation },
    { data_filter::DATA_COMPENSATED_ACCELERATION, filter_decimation },
  }};

  if(commands_3dm::writeFilterMessageFormat(*imu_device, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
    printf("ERROR: Could not set filter message format!\n");

    //
    //Setup the sensor to vehicle rotation
    //
	  sensor_to_vehicle_rotation_euler[0]=0.0;
	  sensor_to_vehicle_rotation_euler[0]=0.0;
	  sensor_to_vehicle_rotation_euler[0]=0.0;

    if(commands_filter::writeSensorToVehicleRotationEuler(*imu_device, sensor_to_vehicle_rotation_euler[0], sensor_to_vehicle_rotation_euler[1], sensor_to_vehicle_rotation_euler[2]) != CmdResult::ACK_OK)
        printf("ERROR: Could not set sensor-2-vehicle rotation!\n");

  //
  //Enable filter auto-initialization
  //

  if(commands_filter::writeAutoInitControl(*imu_device, 1) != CmdResult::ACK_OK)
      printf("ERROR: Could not set filter autoinit control!\n");



  //
  //Reset the filter (note: this is good to do after filter setup is complete)
  //

  if(commands_filter::reset(*imu_device) != CmdResult::ACK_OK)
      printf("ERROR: Could not reset the filter!\n");


  //
  // Register data callbacks
  //

  //Sensor Data

  imu_device->registerExtractor(sensor_data_handlers[0], &sensor_gps_time);
  imu_device->registerExtractor(sensor_data_handlers[1], &sensor_accel);
  imu_device->registerExtractor(sensor_data_handlers[2], &sensor_gyro);

  //Filter Data

  imu_device->registerExtractor(filter_data_handlers[0], &filter_gps_time);
  imu_device->registerExtractor(filter_data_handlers[1], &filter_status);
  imu_device->registerExtractor(filter_data_handlers[2], &filter_euler_angles);
  imu_device->registerExtractor(filter_data_handlers[3], &filter_comp_angular_rate);
  imu_device->registerExtractor(filter_data_handlers[4], &filter_comp_accel);
 
  //
  //Resume the device
  //

  if(commands_base::resume(*imu_device) != CmdResult::ACK_OK)
      printf("ERROR: Could not resume the device!\n");

	//
	// Wait for filter to enter running mode.
	//
	
  while(!filter_state_running){			
    utils->device->update();  
		if(   (filter_status.filter_state == data_filter::FilterMode::GX5_RUN_SOLUTION_ERROR)
			 || (filter_status.filter_state == data_filter::FilterMode::GX5_RUN_SOLUTION_VALID))
		{
		    printf("NOTE: Filter has entered running mode.\n");
		    filter_state_running = true;
		}
  }

	ROS_INFO("IMU setup done");
	
}


void SensorModuleBiped::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  //sub1_ = ros_node.subscribe("/sensor_module", 10, &SensorModuleBiped::topicCallback, this);

  /* publisher */
  pub1_rfoot = ros_node.advertise<std_msgs::UInt16MultiArray>("/rfoot_forces", 10, true);
  pub1_gyro  = ros_node.advertise<std_msgs::Float32MultiArray>("/gyro", 10, true);

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
  
  /* Gyro data */
  
  utils->device->update();

	std_msgs::Float32MultiArray gyro;
	gyro.data.resize(3);

	for(int i=0; i<3; i++)
	  gyro.data[i] = filter_comp_angular_rate.gyro[i];
	  
	//sensors["gyro"] = (float)gyro.data[0];
	//printf("%g\n",sensors["gyro"]);
		
	pub1_gyro.publish(gyro);
	

  result_["gyro_x"] =  gyro.data[0];
  result_["gyro_y"] = -gyro.data[1];
  result_["gyro_z"] = -gyro.data[2];
}

