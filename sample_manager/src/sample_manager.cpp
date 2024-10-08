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
 * sample_manager.cpp
 *
 *  Created on: 2020 Dec.16
 *      Author: Kajita (based on thormang3_manager.cpp)
 */

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "sensor_module_biped/sensor_module_biped.h"

/* Motion Module Header */
#include "thormang3_base_module/base_module.h"
#include "thormang3_action_module/action_module.h"
#include "motion_module_lleg/motion_module_lleg.h"
#include "motion_module_rleg/motion_module_rleg.h"
#include "motion_module_biped/motion_module_biped.h"

//#include "thormang3_manipulation_module/manipulation_module.h"
//#include "thormang3_walking_module/walking_module.h"
//#include "thormang3_gripper_module/gripper_module.h"

using namespace thormang3;
using namespace ROBOTIS;
using namespace mip;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Sample_Manager");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    /* gazebo simulation */
    controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);

    /* Add Sensor Module */
    controller->addSensorModule((robotis_framework::SensorModule*)SensorModuleBiped::getInstance());

    /* Add Motion Module */
    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)ActionModule::getInstance());
    
  	if(robot_file.find("rleg.robot") != std::string::npos){
	    controller->addMotionModule((robotis_framework::MotionModule*)MotionModuleRleg::getInstance());
	  }  		
  	else if(robot_file.find("lleg.robot") != std::string::npos){
	    controller->addMotionModule((robotis_framework::MotionModule*)MotionModuleLleg::getInstance());
	  }
	  else if(robot_file.find("biped.robot") != std::string::npos){
	    controller->addMotionModule((robotis_framework::MotionModule*)MotionModuleBiped::getInstance());
	  }  		
     
    ROS_INFO("Start timer, control cycle= %d [ms]",controller->robot_->getControlCycle() );
    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }

    return 0;
}
