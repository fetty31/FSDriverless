/*!
 ******************************************************************************
 **  CarMaker ROS Interface base class
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#include "cmrosutils/cmrosif.hpp"

#include <cmath>

// CarMaker headers
// Used for CarMaker remote GUI control
#include "GuiCmd.h"

/**
 * @brief STRINGIZE_MACRO to expand and stringize a preprocessor definition
 * @param x the preprocessor definition
 * @return stringized preprocessor definition
 */
#define STRINGIZE_MACRO(x) STRINGIZE(x)
#define STRINGIZE(x) #x

using cm_ros::CarMakerROSInterface;
using CMJob::Log;

CarMakerROSInterface::CarMakerROSInterface() {}

const char* CarMakerROSInterface::getCMVersion() const {
  return STRINGIZE_MACRO(CARMAKER_VER);
}

const char* CarMakerROSInterface::getROSDistro() const {
  return STRINGIZE_MACRO(ROS_DISTRO);
}

int CarMakerROSInterface::init(struct tInfos* infofile_ptr) {
  // use CarMaker Logging
  Log::setLog(Log);
  Log::setLogWarn(LogWarnF);
  Log::setLogErr(LogErrF);

  Log::printLog("CarMaker ROS Node compiled for use with:");
  Log::printLog("  -> ROS Distro = " + std::string(getROSDistro()));
  Log::printLog("  -> CM Version = " + std::string(getCMVersion()));
  Log::printLog("  -> IF Version = " + getInterfaceVersion());

  // read arguments from Infofile and initialize node
  if (!rclcpp::ok()) {
    std::string args = iGetStr(infofile_ptr, "Cfg.Args");
    std::vector<std::string> vec_args;
    auto start = 0UL;
    std::string delim = " ";
    auto end = args.find(delim);
    while (end != std::string::npos) {
      vec_args.push_back(args.substr(start, end - start));
      start = end + delim.length();
      end = args.find(delim, start);
    }
    vec_args.push_back(args.substr(start, end - start));

    char** argv = nullptr;
    auto argc = static_cast<int>(vec_args.size());
    argv = static_cast<char**>(calloc(vec_args.size() + 1, sizeof(*argv)));
    for (size_t i = 0; i < vec_args.size(); i++) {
      argv[i] = strdup(vec_args[i].c_str());
    }

    rclcpp::init(argc, argv);

    for (int i = 0; i < argc; i++) {
      if (argv[i]) free(argv[i]);
    }
    if (argv) free(argv);
  }

  nhp_ = rclcpp::Node::make_shared("cm_node");

  use_sim_time_ =
      static_cast<bool>(iGetIntOpt(infofile_ptr, "Node.UseSimTime", 1));
  if (use_sim_time_) {
    Log::printLog("  -> Providing simulation time!");
    nhp_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    clock_cycle_time_ = iGetIntOpt(infofile_ptr, "Node.nCyclesClock", 10);

    // set up clock server
    typedef CMJob::RosPublisher<rosgraph_msgs::msg::Clock> clock_t;
    auto job = std::make_shared<clock_t>(nhp_, "/clock");
    job->setCycleTime(static_cast<unsigned long>(clock_cycle_time_));
    job->skipFirstCycles(1);
    job->setCallbackHook(CMJob::CallbackHook::PreIn);
    job->registerCallback([](rosgraph_msgs::msg::Clock& msg) {
      auto sim_time =
          static_cast<int64_t>(1e6 * std::round(1e3 * SimCore.Time));
      msg.clock = rclcpp::Time(sim_time, RCL_ROS_TIME);
    });
    scheduler_.addJob(job);
  } else {
    Log::printLog("  -> Not providing simulation time!");
    clock_cycle_time_ = 10;
  }

  auto user_rv = userInit();

  Log::printLog("  -> Creating service '/remote_ctrl'");
  srv_remote_ctrl_ = nhp_->create_service<cmrosutils::srv::CMRemoteControl>(
      "remote_ctrl", std::bind(&CarMakerROSInterface::srvCMRemoteControl, this,
                               std::placeholders::_1, std::placeholders::_2,
                               std::placeholders::_3));

  Log::printLog("Initialization of ROS Node finished!");
  Log::printLog("  -> Node Name = " + std::string(nhp_->get_name()));
  Log::printLog("  -> Namespace = " + std::string(nhp_->get_namespace()));

  return user_rv;
}

bool CarMakerROSInterface::srvCMRemoteControl(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<cmrosutils::srv::CMRemoteControl::Request> req,
    const std::shared_ptr<cmrosutils::srv::CMRemoteControl::Response> resp) {
  (void)header;
  Log::printLog(std::string(nhp_->get_name()) + ": Service '" +
                srv_remote_ctrl_->get_service_name() + "' was triggered with");
  Log::printLog("  type = '" + req.get()->type + "',");
  Log::printLog("  msg = '" + req.get()->msg + "',");
  Log::printLog("  data = '" + req.get()->data + "'");

  std::string default_testrun = "Examples/BasicFunctions/Movie/Benchmark";

  if (req.get()->type == "guicmd") {
    /* Commands to CarMaker GUI
     * - Using ScriptControl/tcl commands
     * - More information see "ProgrammersGuide chapter ScriptControl"
     */
    if (req.get()->msg == "eval") {
      // Example for data string:
      // 'LoadTestRun Examples/BasicFunctions/Movie/Benchmark; StartSim'
      resp->res = GuiCmd_Eval(req.get()->data.c_str());
    } else {
      if (req.get()->msg == "start") {
        std::string cmd = "LoadTestRun " + default_testrun + "; StartSim";
        if (!req.get()->data.empty()) cmd = req.get()->data + "; StartSim";
        resp->res = GuiCmd_Eval(cmd.c_str());
      } else if (req.get()->msg == "stop")
        resp->res = GuiCmd_Eval("StopSim");
    }
  } else if (req.get()->type == "cmd") {
    /* Commands directly to CarMaker executable
     * Warning: Information normally provided by CarMaker GUI might be missing
     */
    if (req.get()->msg == "start") {
      std::string testrun =
          req.get()->data.empty() ? default_testrun : req.get()->data;
      // Most strings are already provided by CarMaker GUI
      SimStart(nullptr, nhp_->get_name(), testrun.c_str(), nullptr, nullptr);
    } else if (req.get()->msg == "stop") {
      SimStop2(0);
    }
    resp->res = 0;
  }
  return true;
}

void CarMakerROSInterface::declQuants() { userDeclQuants(); }

int CarMakerROSInterface::testrunStartAtBegin(struct tInfos* infofile_ptr) {
  if (infofile_ptr) {
    int node_mode = iGetIntOpt(infofile_ptr, "Node.Mode",
                               static_cast<int>(SyncMode::kDisabled));
    int sync_mode = iGetIntOpt(infofile_ptr, "Node.Sync.Mode",
                               static_cast<int>(SyncMode::kDisabled));
    node_mode_ = static_cast<NodeMode>(node_mode);
    sync_mode_ = static_cast<SyncMode>(sync_mode);
  }

  if (!SimCore.IsRegularInit || !infofile_ptr ||
      node_mode_ == NodeMode::kDisabled) {
    node_mode_ = NodeMode::kDisabled;
    Log::printLog("CarMaker ROS Node disabled!");
    return 0;
  }

  scheduler_.resetJobs();

  {
    std::stringstream ss;
    ss << "CarMaker ROS Node enabled: ";
    ss << "Mode = " << std::to_string(static_cast<uint8_t>(node_mode_)) << ", ";
    ss << "SyncMode = " << std::to_string(static_cast<uint8_t>(sync_mode_));

    Log::printLog(ss.str());
    Log::printLog("  -> Node Name = " + std::string(nhp_->get_name()));
  }

  if (sync_mode_ != SyncMode::kDisabled && sync_mode_ != SyncMode::kTopic) {
    std::stringstream ss;
    ss << "CMNode: Invalid synchronization mode ";
    ss << "'" << std::to_string(static_cast<uint8_t>(sync_mode_)) << "'!";
    Log::printError(EC_Sim, ss.str());

    sync_mode_ = SyncMode::kDisabled;
    return -1;
  }

  max_sync_time_ = iGetDblOpt(infofile_ptr, "Node.Sync.TimeMax", 1.0);
  rel_cycle_num_ = 0;

  if (use_sim_time_) {
    int cycle_time = iGetIntOpt(infofile_ptr, "Node.nCyclesClock", 10);
    if (cycle_time > 0) {
      clock_cycle_time_ = cycle_time;
    }
    // Reset /clock
    scheduler_.getJob("/clock")->execute();
    std::stringstream ss;
    ss << "  -> Publish /clock every " << cycle_time << "ms";
    Log::printLog(ss.str());
  }

  return userTestrunStartAtBegin();
}

int CarMakerROSInterface::testrunStartAtEnd() {
  if (node_mode_ == NodeMode::kDisabled) {
    return 1;
  }

  auto user_rv = userTestrunStartAtEnd();

  // Advertised and subscribed Topics
  auto topics = nhp_->get_topic_names_and_types();
  Log::printLog("  -> Advertised and subscribed topics (" +
                std::to_string(topics.size()) + "):");
  for (auto const& topic : topics) {
    Log::printLog("    -> " + topic.first);
  }

  return user_rv;
}

int CarMakerROSInterface::testrunRampUp() {
  if (node_mode_ == NodeMode::kDisabled) {
    return 1;
  }

  return userTestrunRampUp();
}

void CarMakerROSInterface::in() {
  if (node_mode_ == NodeMode::kDisabled) {
    // still process messages/services in disabled mode
    rclcpp::spin_some(nhp_);
    return;
  }

  scheduler_.executeJobs(CMJob::CallbackHook::PreIn, rel_cycle_num_);

  if (sync_mode_ == SyncMode::kTopic) {
    scheduler_.lock(rel_cycle_num_, [&]() { rclcpp::spin_some(nhp_); });
  } else {
    rclcpp::spin_some(nhp_);
  }

  userIn();

  scheduler_.executeJobs(CMJob::CallbackHook::In, rel_cycle_num_);
}

int CarMakerROSInterface::drivmanCalc(const double& dt) {
  if (node_mode_ == NodeMode::kDisabled || SimCore.State != SCState_Simulate) {
    return 0;
  }

  scheduler_.executeJobs(CMJob::CallbackHook::DrivMan, rel_cycle_num_);

  return userDrivmanCalc(dt);
}

int CarMakerROSInterface::vehicleControlCalc(const double& dt) {
  if (node_mode_ == NodeMode::kDisabled || SimCore.State != SCState_Simulate) {
    return 0;
  }

  scheduler_.executeJobs(CMJob::CallbackHook::VehicleControl, rel_cycle_num_);

  return userVehicleControlCalc(dt);
}

int CarMakerROSInterface::calc(const double& dt) {
  if (node_mode_ == NodeMode::kDisabled || SimCore.State != SCState_Simulate) {
    return 0;
  }

  scheduler_.executeJobs(CMJob::CallbackHook::Calc, rel_cycle_num_);

  return userCalc(dt);
}

void CarMakerROSInterface::out() {
  if (node_mode_ == NodeMode::kDisabled || SimCore.State != SCState_Simulate) {
    return;
  }

  userOut();

  scheduler_.executeJobs(CMJob::CallbackHook::Out, rel_cycle_num_);
  scheduler_.updateJobs(rel_cycle_num_);
  rel_cycle_num_++;
}

int CarMakerROSInterface::testrunEnd() {
  node_mode_ = NodeMode::kDisabled;

  return userTestrunEnd();
}

int CarMakerROSInterface::end() {
  auto user_rv = userEnd();

  scheduler_.deleteJobs();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return user_rv;
}

int CarMakerROSInterface::userInit() { return 1; }

void CarMakerROSInterface::userDeclQuants() {}

int CarMakerROSInterface::userTestrunStartAtBegin() { return 1; }

int CarMakerROSInterface::userTestrunStartAtEnd() { return 1; }

int CarMakerROSInterface::userTestrunRampUp() { return 1; }

void CarMakerROSInterface::userIn() {}

int CarMakerROSInterface::userDrivmanCalc(const double& dt) {
  (void)dt;  // suppresses -Wunused-parameter

  return 1;
}

int CarMakerROSInterface::userVehicleControlCalc(const double& dt) {
  (void)dt;  // suppresses -Wunused-parameter

  return 1;
}

int CarMakerROSInterface::userCalc(const double& dt) {
  (void)dt;  // suppresses -Wunused-parameter

  return 1;
}

void CarMakerROSInterface::userOut() {}

int CarMakerROSInterface::userTestrunEnd() { return 1; }

int CarMakerROSInterface::userEnd() { return 1; }
