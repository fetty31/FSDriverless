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

#ifndef CMNODE_HELLOCM_HPP
#define CMNODE_HELLOCM_HPP

#include "cmrosutils/cmrosif.hpp"
#include "hellocm_msgs/msg/cm2_ext.hpp"
#include "hellocm_msgs/msg/ext2_cm.hpp"
#include "hellocm_msgs/srv/init.hpp"

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
   * @brief ext2cmCallback Callback function for ext2cm subscriber.
   * Used in this example to synchronize the CM cycle to
   * @param msg Received ROS message
   */
  void ext2cmCallback(hellocm_msgs::msg::Ext2CM::ConstSharedPtr msg);

  /**
   * @brief cm2extFillMsg prepares the message to be sent to the external node.
   * Demonstrates how CarMaker variables can be sent out as ros messages.
   * @param msg the actual message being prepared by this function
   */
  void cm2extFillMsg(hellocm_msgs::msg::CM2Ext& msg);

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

/**
 * @brief CMNodeHelloCM_customFunction C-interface wrapper for custom user
 * function to be accessed in other sources, e.g. User.c:
 * Use 'CMCppIFLoader_getSymbol("CMNodeHelloCM_customFunction")' to get the
 * symbol.
 * Use 'CMCppIFLoader_getInterfacePtr()' to get instantiated interface object
 * See "lib/cmcppifloader.h" for more information.
 * @param intf void pointer to instantiated CMNodeHelloCM object
 * @param input some constant integer
 * @param output string to be filled by CMNodeHelloCM::customFunction
 * @return value returned by CMNodeHelloCM::customFunction
 */
extern "C" int CMNodeHelloCM_customFunction(void* intf, const int input,
                                            char* output);

#endif  // CMNODE_HELLOCM_HPP
