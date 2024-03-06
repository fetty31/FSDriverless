/*!
 ******************************************************************************
 **  CarMaker ROS Interface base class
 **  Unsupported example provided by IPG free of charge
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#ifndef CM_ROS_UTILS_CMROSIF_HPP
#define CM_ROS_UTILS_CMROSIF_HPP

// ROS header
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

// CarMaker ROS Utils header
#include "cmcppif.h"
#include "cmrosutils/CMJob.h"
#include "cmrosutils/cmjob_publisher.hpp"
#include "cmrosutils/cmjob_subscriber.hpp"
#include "cmrosutils/srv/cm_remote_control.hpp"

// CarMaker header
#include "DataDict.h"
#include "InfoUtils.h"
#include "Log.h"
#include "SimCore.h"

namespace node_mode {
/**
 * @brief The NodeMode enum Enables or disables the CarMaker node
 */
enum class NodeMode {
  // node is disabled: No publishing etc.
  kDisabled = 0,

  // node is enabled, spinOnce is used
  kDefault = 1
};
}  // namespace node_mode

namespace sync_mode {
/**
 * @brief The SyncMode enum to manage synchronization between CarMaker and
 * external nodes
 */
enum class SyncMode {
  // no synchronization with CarMaker
  kDisabled = 0,

  // synchronize on incoming messages on specified topics
  kTopic = 1
};
}  // namespace sync_mode

namespace cm_ros {

typedef node_mode::NodeMode NodeMode;
typedef sync_mode::SyncMode SyncMode;

using cmcppif::CarMakerCPPInterface;

/**
 * @brief The CarMakerROSInterface base class extends the CarMaker C++ Interface
 * and implements the basic CarMaker ROS node functioniality.
 */
class CarMakerROSInterface : public CarMakerCPPInterface {
 public:
  CarMakerROSInterface();

  /**
   * @brief getInterfaceVersion is used to check whether this interface is
   * compatible with the loader module.
   * @return Version string of this interface
   */
  const std::string& getInterfaceVersion(void) const final {
    return cmcppif::INTERFACE_VERSION;
  }

  /**
   * @brief getCMVersion is used to print the CarMaker version that the
   * interface was compiled for.
   * @return CarMaker version string e.g. "10.0.1"
   */
  const char* getCMVersion() const final;

  /**
   * @brief getROSDistro is used to print the ROS distribution name that the
   * interface was compiled for.
   * @return
   */
  const char* getROSDistro() const;

  /**
   * @brief init initializes the node according to the settings in the Infofile.
   * Evaluates the node arguments for e.g. argument remapping.
   * Sets up the clock server if #use_sim_time_ is true.
   * Calls userInit() and prints general information about the node state.
   * @param infofile_ptr pointer to CMRosIFParameters Infofile
   * @return return value obtained by userInit()
   */
  int init(struct tInfos* infofile_ptr) final;

  /**
   * @brief declQuants calls userDeclQuants()
   */
  void declQuants() final;

  /**
   * @brief testrunStartAtBegin reads and applies node and synchronization
   * settings from Infofile.
   * Resets the clock if #use_sim_time_ is true.
   * @param infofile_ptr pointer to CMRosIFParameters Infofile
   * @return 0 if the node is disabled, -1 if an invalid sync mode was
   * requested, otherwise the return value obtained by userTestrunStartAtBegin()
   */
  int testrunStartAtBegin(struct tInfos* infofile_ptr) final;

  /**
   * @brief testrunStartAtEnd resets all registered jobs, calls
   * userTestrunStartAtEnd() and prints information about advertised and
   * subscribed topics.
   * @return 1 if the node is disabled, otherwise the return value obtained by
   * userTestrunStartAtEnd()
   */
  int testrunStartAtEnd() final;

  /**
   * @brief testrunRampUp calls userTestrunRampUp()
   * @return 1 if the node is disabled, otherwise the return value obtained by
   * userTestrunRampUp()
   */
  int testrunRampUp() final;

  /**
   * @brief in publishes the clock every #clock_cycle_time_, waits for
   * synchronized topics in activated #sync_mode_, calls userIn() and executes
   * all registered jobs associated with CMJob::CallbackHook::PreIn and
   * CMJob::CallbackHook::In that are due in the current cycle
   */
  void in() final;

  /**
   * @brief drivmanCalc executes all registered jobs associated with
   * CMJob::CallbackHook::DrivMan that are due in the current cycle and calls
   * userDrivmanCalc()
   * @param dt the simulation time step
   * @return 0 if the node is disabled, otherwise the return value obtained by
   * userDrivmanCalc()
   */
  int drivmanCalc(const double& dt) final;

  /**
   * @brief vehicleControlCalc executes all registered jobs associated with
   * CMJob::CallbackHook::VehicleControl that are due in the current cycle and
   * calls userVehicleControlCalc()
   * @param dt the simulation time step
   * @return 0 if the node is disabled, otherwise the return value obtained by
   * userVehicleControlCalc()
   */
  int vehicleControlCalc(const double& dt) final;

  /**
   * @brief calc executes all registered jobs associated with
   * CMJob::CallbackHook::Calc that are due in the current cycle and calls
   * userCalc()
   * @param dt the simulation time step
   * @return 0 if the node is disabled, otherwise the return value obtained by
   * userCalc()
   */
  int calc(const double& dt) final;

  /**
   * @brief out calls userOut(), executes all registered jobs associated with
   * CMJob::CallbackHook::Out that are due in the current cycle, updates all
   * registered jobs and increments #rel_cycle_num_.
   */
  void out() final;

  /**
   * @brief testrunEnd disables the node and calls userTestrunEnd()
   * @return return value obtained by userTestrunEnd()
   */
  int testrunEnd() final;

  /**
   * @brief end calls userEnd() and shuts down the node
   * @return return value obtained by userEnd()
   */
  int end() final;

 protected:
  /**
   * @brief userInit basic initialization of user specific code, e.g.
   * initializing parameters and setting up subscriber and publisher that aren't
   * dependent on the loaded TestRun.
   * @sa CarMakerCPPInterface::init()
   * @return < 0 if initialization fails and simulation should abort, >= 0
   * otherwise
   */
  virtual int userInit();

  /**
   * @brief userDeclQuants add user specific quantities to the dictionary
   * @sa CarMakerCPPInterface::declQuants()
   */
  virtual void userDeclQuants();

  /**
   * @brief userTestrunStartAtBegin initialize and/or reset user specific code
   * before parameters are read in for a new simulation.
   * @sa CarMakerCPPInterface::testrunStartAtBegin()
   * @return < 0 if procedure fails and simulation should abort, >= 0 otherwise
   */
  virtual int userTestrunStartAtBegin();

  /**
   * @brief userTestrunStartAtEnd initialize and/or reset user specific code
   * after parameters are read in for a new simulation, e.g. like setting up
   * publishers for sensors.
   * @sa CarMakerCPPInterface::testrunStartAtEnd()
   * @return < 0 if procedure fails and simulation should abort, >= 0 otherwise
   */
  virtual int userTestrunStartAtEnd();

  /**
   * @brief userTestrunRampUp perform a smooth transition of user variables.
   * Hook is called repeatedly until all interfaces and other modules return
   * true.
   * @sa CarMakerCPPInterface::testrunRampUp()
   * @return false if node is not ready, true if node is ready
   */
  virtual int userTestrunRampUp();

  /**
   * @brief userIn in called in the main loop in realtime context, at the very
   * beginning of the CarMaker cycle. Use it for e.g. subscribing to topics.
   * @sa CarMakerCPPInterface::in()
   */
  virtual void userIn();

  /**
   * @brief userDrivmanCalc called in realtime context, after driving maneuver
   * calculation
   * @sa CarMakerCPPInterface::drivmanCalc()
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int userDrivmanCalc(const double& dt);

  /**
   * @brief userVehicleControlCalc called in realtime context, after vehicle
   * control calculation
   * @sa CarMakerCPPInterface::vehicleControlCalc()
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int userVehicleControlCalc(const double& dt);

  /**
   * @brief userCalc called in realtime context, after vehicle model has been
   * calculated
   * @sa CarMakerCPPInterface::calc()
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int userCalc(const double& dt);

  /**
   * @brief userOut called in the main loop in realtime context, close to end of
   * CarMaker cycle. Use it for e.g. publishing messages.
   * @sa CarMakerCPPInterface::out()
   */
  virtual void userOut();

  /**
   * @brief userTestrunEnd to e.g. write data to file or clean up after the
   * simulation.
   * @sa CarMakerCPPInterface::testrunEnd()
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int userTestrunEnd();

  /**
   * @brief userEnd to clean up before the application stops.
   * @sa CarMakerCPPInterface::end()
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int userEnd();

  /**
   * @brief nhp_ ROS node handle pointer
   */
  rclcpp::Node::SharedPtr nhp_;

  /**
   * @brief scheduler_ JobScheduler singleton to handle several jobs, most
   * importantly publisher and synchronized subscriber
   */
  CMJob::JobScheduler& scheduler_ = CMJob::JobScheduler::instance();

  /**
   * @brief node_mode_ Mode of CarMaker ROS node to dis-/enable the node
   */
  node_mode::NodeMode node_mode_;

  /**
   * @brief sync_mode_ Mode to manage synchronization between CarMaker and
   * external nodes
   */
  sync_mode::SyncMode sync_mode_;

  /**
   * @brief rel_cycle_num_ CarMaker relative cycle number. Increments at the end
   * of each cycle and gets reset each TestRun.
   */
  uint64_t rel_cycle_num_;

  /**
   * @brief max_sync_time_ Max. seconds the simulation should wait for a delayed
   * message to arrive on synchronized topic before the simulation gets aborted
   */
  double max_sync_time_;

  /**
   * @brief clock_cycle_time_ Cycle time of the clock publisher
   */
  int clock_cycle_time_;

 private:
  /**
   * @brief srvCMRemoteControl Callback function for CM remote control service.
   * Takes either ScriptControl commands for the CM GUI or simple commands
   * directly to the CM executable for starting and stopping a TestRun.
   * @param req Received message with (GUI) commands by the service call
   * @param resp Return value of the issued (GUI) command
   * @return Always true
   */
  bool srvCMRemoteControl(
      const std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<cmrosutils::srv::CMRemoteControl::Request> req,
      const std::shared_ptr<cmrosutils::srv::CMRemoteControl::Response> resp);

  /**
   * @brief srv_remote_ctrl_ Basic CarMaker remote control service server
   */
  rclcpp::Service<cmrosutils::srv::CMRemoteControl>::SharedPtr srv_remote_ctrl_;

  /**
   * @brief use_sim_time_ Set to true if CarMaker should provide the simulation
   * time on the /clock topic.
   */
  bool use_sim_time_;
};
}  // namespace cm_ros
#endif  // CM_ROS_UTILS_CMROSIF_HPP
