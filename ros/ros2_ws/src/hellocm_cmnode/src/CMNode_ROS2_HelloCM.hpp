/*!
 ******************************************************************************
 **  Example for a CarMaker ROS Node that communicates with an external node
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#ifndef CMNODE_ROS2_HELLOCM_HPP
#define CMNODE_ROS2_HELLOCM_HPP

/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */
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
#include "infoc.h"
#include <math.h>
#include "Car/Car.h"
#include "Car/Brake.h"
#include "DrivMan.h"
#include "VehicleControl.h"
#include "Traffic.h"

/* ROS */
#include "cmrosutils/cmrosif.hpp"                   /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/srv/cm_remote_control.hpp"     /* Basic service for CarMaker remote from ROS */
#include "sensor_msgs/msg/point_cloud.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <angles/angles.h>
#include "vehiclecontrol_msgs/msg/vehicle_control.hpp"
#include "camera_msgs/msg/camera_detection_array.hpp"

/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 0
//#  include "hellocm/ROS1_HelloCM.h"  /* External ROS Node. Topic name, ... */
#else
  #include "hellocm_msgs/msg/cm2_ext.hpp"
  #include "hellocm_msgs/msg/ext2_cm.hpp"
  #include "hellocm_msgs/srv/init.hpp"
#endif

namespace cm_ros {
/**
 * @brief The CMNodeHelloCM class serves as an example for a CarMaker ROS Node
 * that communicates with an external node. It derives from the
 * CarMakerROSInterface base class, which already implements the basic CarMaker
 * ROS node functioniality.
 */
class CMNodeHelloCM : public CarMakerROSInterface {
 public:
  CMNodeHelloCM();

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
   * @brief ext2cmCallback Callback function for ext2cm subscriber.
   * Used in this example to synchronize the CM cycle to
   * @param msg Received ROS message
   */
  void ext2cmCallback(hellocm_msgs::msg::Ext2CM::ConstSharedPtr msg);

  /** 
   * @brief vehicleControlCallback Vehicle Control subcriber callback when receiving a VC message.
   * @param msg Received ROS Vehicle Control message.
   */
  void vehicleControlCallback(vehiclecontrol_msgs::msg::VehicleControl::ConstSharedPtr msg);

  /**
   * @brief cm2extFillMsg prepares the message to be sent to the external node.
   * Demonstrates how CarMaker variables can be sent out as ros messages.
   * @param msg the actual message being prepared by this function
   */
  void cm2extFillMsg(hellocm_msgs::msg::CM2Ext& msg);

  /**
   * @brief pointcloudFillMsg prepares the message of LidarRSI pointclouid data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being prepared by this function
   */
  void pointcloudFillMsg(sensor_msgs::msg::PointCloud& msg);

  /**
   * @brief objectlistFillMsg prepares the message of Object sensor ObjectList data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being prepared by this function
   */
  void objectlistFillMsg(visualization_msgs::msg::MarkerArray& msg);

  /**
   * @brief objectlistFillMsg prepares the message of Camera data 
   * to be transmitted from CarMaker over ROS.
   * @param msg the actual message being prepared by this function
   */
  void cameraFillMsg(camera_msgs::msg::CameraDetectionArray& msg);

  /**
   * @brief init_ service client to demonstrate ros service calls to an
   * external node. In this example the service is called at each TestRun start
   * resetting the external node.
   */
  rclcpp::Client<hellocm_msgs::srv::Init>::SharedPtr srv_init_;

  /**
   * @brief param_client_ synchronous parameter client to retrieve parameters
   * from external node.
   */
  rclcpp::SyncParametersClient::SharedPtr param_client_;

  /**
   * @brief synth_delay_ Synthetic delay in seconds to artificially delay the
   * external node to showcase synchronization mode
   */
  double synth_delay_;
};
}  // namespace cm_ros

#endif  // CMNODE_ROS2_HELLOCM_HPP
