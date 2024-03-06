/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

/*!
 ******************************************************************************
                Main CarMaker ROS Node for BCN eMotorsport
 ******************************************************************************
 */

#ifndef CMNODE_H
#define CMNODE_H

#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <math.h>
#include <memory>
#include <boost/exception/diagnostic_information.hpp> 

#include "utils/params.hh"

#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"

//CarMaker Header File Includes
#include "Vehicle.h"
#include "Vehicle/Sensor_LidarRSI.h"
#include "Vehicle/Sensor_Object.h"
#include "Vehicle/Sensor_Camera.h"
#include "Vehicle/Sensor_Inertial.h"
#include "infoc.h"
#include "Car/Car.h"
#include "Car/Brake.h"
#include "DrivMan.h"
#include "VehicleControl.h"
#include "Traffic.h"
#include "Car/PowerTrain.h"

/* ROS */
#include "cmrosutils/cmrosif.h"               /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemoteControl.h"       /* Basic service for CarMaker remote from ROS */
#include "tf/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <angles/angles.h>

#include "ipg_msgs/CarState.h"
#include "ipg_msgs/CarCommands.h"

#include "nav_msgs/Odometry.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Imu.h"

#include "ipg_msgs/GeneralSensor.h"
#include "ipg_msgs/MotorOutput.h"
#include "ipg_msgs/MotorInput.h"
#include "ipg_msgs/BatteryStatus.h"
#include "ipg_msgs/KinEstimate.h"
#include "ipg_msgs/DynEstimate.h"

#include "std_msgs/Bool.h"

#include "tracker/Init.h"

#include "utils/noise.hh"

namespace cm_ros {
/**
 * @brief The CMNode class derives from the
 * CarMakerROSInterface base class, which already implements the basic CarMaker
 * ROS node functioniality.
 */
class CMNode : public CarMakerROSInterface {
 public:
  CMNode();       // Constructor
  ~CMNode();      // Destructor

  std::shared_ptr<cm_yaml::Params> paramsPtr; // Params struct ptr

  noise::Gaussian* noise; // Noise class

  double initial_heading; // Used for correcting the initial heading -> we always want to start at 0.0 of heading (base_link == global at t=0)

  double initial_pos[3]; // Used for starting always at 0,0,0 -> it isn't strictly necessary but feels nice

  bool headingFlag, posFlag, finishFlag; // Flags

  int Ninertial;  // Number of inertial sensors available

  /**
   * @brief customFunction to demonstrate possibility to exchange
   * data between interface and CarMaker executable
   * @param input some constant integer
   * @param output demangled/prettified function name
   * @return ROS /clock cycle time + provided input integer
   */
  int customFunction(const int input, char* output);

 private:
  /**
   * @brief userInit sets up the ros publisher job and the service client
   * @return 1
   */
  int userInit() final;

  /**
   * @brief userDeclQuants declares some User Accessible Quantities (UAQs) for
   * data storage in ERG files, data access via e.g. DVA or visualization in
   * e.g. IPGControl
   */
  void userDeclQuants() final;

  /**
   * @brief userTestrunStartAtBegin first calls the service of the external node
   * to resets it. Then it sets up the ros subscriber job. In case of
   * synchronized mode the job uses the cycle time of the external node
   * retrieved via ros parameter server and checks whether it is compatible with
   * the current clock cycle time.
   * @return 1 if successful, -1 if otherwise
   */
  int userTestrunStartAtBegin() final;

  /**
   * @brief userTestrunEnd deletes the subscriber job
   * @return 1
   */
  int userTestrunEnd() final;

  /**
   * @brief userVehicleControlCalc called in realtime context, after vehicle
   * control calculation
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  int userVehicleControlCalc(const double& dt);

  /**
   * @brief userCalc called in realtime context, after vehicle
   * model calculation
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  int userCalc(const double& dt);

  /**
   * @brief CarCommandsCallback Callback function for Car Commands subscriber.
   * @param msg Received AS ROS message
   */
  void CarCommandsCallback(const ipg_msgs::CarCommands::ConstPtr &msg);

  /**
   * @brief SteeringCallback Callback function for Steering subscriber.
   * @param msg Received AS ROS message
   */
  void SteeringCallback(const ipg_msgs::CarCommands::ConstPtr &msg);

  /**
   * @brief Finish Callback function for finish flag subscriber.
   * @param msg Received AS ROS message
   */
  void FinishCallback(const std_msgs::Bool::ConstPtr &msg);

  /**
   * @brief MotorCallback Callback function for Motor subscriber.
   * @param msg Received Motor ROS message
   * @param pos the chosen motor: [0, 1, 2, 3] == [FL, FR, RL, RR]
   */
  void MotorCallback(const ipg_msgs::MotorInput::ConstPtr &msg, unsigned int pos);

    /**
   * @brief CarStateFillMsg prepares the message to be sent to the external node.
   * @param msg the actual message being filled by this function
   */
  void CarStateFillMsg(ipg_msgs::CarState::Ptr& msg);

  /**
   * @brief pointcloudFillMsg prepares the message of LidarRSI pointclouid data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void pointcloudFillMsg(sensor_msgs::PointCloud::Ptr& msg);

  /**
   * @brief OdomFillMsg prepares the message of Vehicle's Odometry data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void OdomFillMsg(nav_msgs::Odometry::Ptr& msg);

  /**
   * @brief OdomFillMsg prepares the message of Vehicle's Odometry data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void OdomFillMsg(ipg_msgs::CarState::Ptr& msg);

  /**
   * @brief IMUFillMsg prepares the message of Vehicle's IMU data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   * @param sbg the chosen IMU sensor: [0, 1, 2] == [CoG, front, rear]
   */
  void IMUFillMsg(sensor_msgs::Imu::Ptr& msg, unsigned int sensor);

  /**
   * @brief BatteryFillMsg prepares the message of battery data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void BatteryFillMsg(ipg_msgs::BatteryStatus::Ptr& msg);

  /**
   * @brief ThrottleFillMsg prepares the message of throttle pedal data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void ThrottleFillMsg(ipg_msgs::GeneralSensor::Ptr& msg);

  /**
   * @brief BrakeFillMsg prepares the message of brake pedal data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void BrakeFillMsg(ipg_msgs::GeneralSensor::Ptr& msg);

  /**
   * @brief PressureFillMsg prepares the message of brake pressure data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void PressureFillMsg(ipg_msgs::GeneralSensor::Ptr& msg);

  /**
   * @brief SteerFillMsg prepares the message of steering position 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void SteerFillMsg(ipg_msgs::GeneralSensor::Ptr& msg);

  /**
   * @brief KinematicFillMsg prepares the message of kinematic estimation 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void KinematicFillMsg(ipg_msgs::KinEstimate::Ptr& msg);

  /**
   * @brief DynamicFillMsg prepares the message of dynamic estimation 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   */
  void DynamicFillMsg(ipg_msgs::DynEstimate::Ptr& msg);

  /**
   * @brief InverterFillMsg prepares the message of inverter data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being filled by this function
   * @param pos the chosen motor: [0, 1, 2, 3] == [FL, FR, RL, RR]
   */
  void InverterFillMsg(ipg_msgs::MotorOutput::Ptr& msg, unsigned int pos);

    /**
   * @brief printSmth prints data using CarMaker Log function -> can be seen in CMmain session log.
   * @param num the actual data wanted to be logged
   * @param comment a comment to log together with num
   */
  template<typename mytype> void printSmth(mytype num, std::string comment = "");

    /**
   * @brief printOnce prints data using CarMaker Log function only once.
   * @param num the actual data wanted to be logged
   * @param comment a comment to log together with num
   */
  template<typename mytype> void printOnce(mytype num, std::string comment = ""){
    static const auto runOnce = [this, num, comment] { this->printSmth<mytype>(num, comment); return true;}();
  }

    /**
   * @brief clockTimeCheck checks whether given cycle_time is bigger than clock time
   * @param cycle_time the actual cycle time
   * @param name job name
   */
  int clockTimeCheck(int cycle_time, const char* name);

    /**
   * @brief correctHeading checks whether given heading is in [-pi, pi]. It also sets heading to 0.0 at first iteration
   * @param heading the actual heading [rad]
   */
  double correctHeading(double heading);

    /**
   * @brief correctPose checks whether given angle is in [-pi, pi].
   * @param phi the angle [rad]
   */
  double correctPose(double phi);

    /**
   * @brief correctSteering transforms wheel frame steering command (sent by MPC) to steering wheel frame [-270º, 270º]. Linear approximation!
   * @param steering the actual steering (in wheel frame) [rad]
   */
  double correctSteering(double steering);

    /**
   * @brief m2w_trq transforms motor frame torque command (sent by CTRL Master) to torque in wheel frame (the one given to IPG).
   * @param trq the actual torque (in motor frame) [Nm]
   */
  double m2w_trq(double trq);

    /**
   * @brief m2w_vel transforms motor frame angular velocity command (sent by CTRL Master) to angular velocity in wheel frame (the one given to IPG).
   * @param vel the actual vel (in motor frame) [rad/s]
   */
  double m2w_vel(double vel);

    /**
   * @brief correctPosition sets position to 0.0 at first iteration and calls rotatePosition()
   * @param x coordinate [m]
   * @param y coordinate [m]
   * @param z coordinate [m]
   */
  void correctPosition(double* x, double* y, double* z);

    /**
   * @brief rotatePosition rotates points in the xy plane counterclockwise
   * @param x coordinate [m]
   * @param y coordinate [m]
   * @param angle the actual rotation angle [rad]
   */
  void rotatePosition(double* x, double* y, double* angle);

      /**
   * @brief pubTF publishes ROS transform from frame to child_frame
   * @param frame the actual parent frame
   * @param child_frame the actual child frame
   * @param x coordinate [m]
   * @param y coordinate [m]
   * @param z coordinate [m]
   * @param yaw the actual heading (or yaw) [rad]
   */
  void pubTF(const std::string& frame, const std::string& child_frame, double* x, double* y, double* z, double* yaw /*== heading*/);

      /**
   * @brief pubTF publishes ROS transform from frame to child_frame
   * @param frame the actual parent frame
   * @param child_frame the actual child frame
   * @param x coordinate [m]
   * @param y coordinate [m]
   * @param z coordinate [m]
   * @param q a quaternion defining rotation pose [rad]
   */
  void pubTF(const std::string& frame, const std::string& child_frame, double* x, double* y, double* z, tf::Quaternion q);

    /**
   * @brief Service client to call an external node server. 
   * In this case the services are called before each TestRun resetting the external node.
   */
  ros::ServiceClient srv_accum_;

  /**
   * @brief synth_delay_ Synthetic delay in seconds to artificially delay the
   * external node to showcase synchronization mode
   */
  double synth_delay_;
};
}  // namespace cm_ros

/**
 * @brief CMNode_customFunction C-interface wrapper for custom user
 * function to be accessed in other sources, e.g. User.c:
 * Use 'CMCppIFLoader_getSymbol("CMNode_customFunction")' to get the
 * symbol.
 * Use 'CMCppIFLoader_getInterfacePtr()' to get instantiated interface object
 * See "lib/cmcppifloader.h" for more information.
 * @param intf void pointer to instantiated CMNode object
 * @param input some constant integer
 * @param output string to be filled by CMNode::customFunction
 * @return value returned by CMNode::customFunction
 */
extern "C" int CMNode_customFunction(void* intf, const int input,
                                            char* output);

#endif  // CMNODE_H
