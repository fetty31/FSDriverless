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

#include "cmrosutils/cmrosif.h"

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
  if (!ros::isInitialized()) {
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

    ros::init(argc, argv, "cm_node", ros::init_options::NoSigintHandler);

    for (int i = 0; i < argc; i++) {
      if (argv[i]) free(argv[i]);
    }
    if (argv) free(argv);
  }

  if (!ros::master::check()) {
    Log::printError(EC_Init,
                    "Can't contact ROS Master!\n Start roscore or run launch "
                    "file e.g. via Extras->CMRosIF\n");
    ros::shutdown();
    return -1;
  }

  nhp_ = boost::make_shared<ros::NodeHandle>();

  use_sim_time_ =
      static_cast<bool>(iGetIntOpt(infofile_ptr, "Node.UseSimTime", 1));
  if (use_sim_time_) {
    Log::printLog("  -> Providing simulation time!");
    nhp_->setParam("/use_sim_time", true);
    clock_cycle_time_ = iGetIntOpt(infofile_ptr, "Node.nCyclesClock", 10);

    // set up clock server
    typedef CMJob::RosPublisher<rosgraph_msgs::Clock::Ptr> clock_t;
    auto job = std::make_shared<clock_t>(nhp_, "/clock");
    job->setCycleTime(static_cast<unsigned long>(clock_cycle_time_));
    job->skipFirstCycles(1);
    job->setCallbackHook(CMJob::CallbackHook::PreIn);
    job->registerCallback([](rosgraph_msgs::Clock::Ptr& msg) {
      msg->clock = ros::Time(SimCore.Time);
    });
    scheduler_.addJob(job);
  } else {
    Log::printLog("  -> Not providing simulation time!");
    clock_cycle_time_ = 10;
  }

  auto user_rv = userInit();

  Log::printLog("  -> Creating service '/remote_ctrl'");
  srv_remote_ctrl_ = nhp_->advertiseService(
      "remote_ctrl", &CarMakerROSInterface::srvCMRemoteControl, this);

  Log::printLog("Initialization of ROS Node finished!");
  Log::printLog("  -> Node Name = " + ros::this_node::getName());
  Log::printLog("  -> Namespace = " + ros::this_node::getNamespace());

  return user_rv;
}

bool CarMakerROSInterface::srvCMRemoteControl(
    cmrosutils::CMRemoteControl::Request& req,
    cmrosutils::CMRemoteControl::Response& resp) {
  Log::printLog(ros::this_node::getName() + ": Service '" +
                srv_remote_ctrl_.getService() + "' was triggered with");
  Log::printLog("  type = '" + req.type + "',");
  Log::printLog("  msg = '" + req.msg + "',");
  Log::printLog("  data = '" + req.data + "'");

  std::string default_testrun = "Examples/BasicFunctions/Movie/Benchmark";

  if (req.type == "guicmd") {
    /* Commands to CarMaker GUI
     * - Using ScriptControl/tcl commands
     * - More information see "ProgrammersGuide chapter ScriptControl"
     */
    if (req.msg == "eval") {
      // Example for data string:
      // 'LoadTestRun Examples/BasicFunctions/Movie/Benchmark; StartSim'
      resp.res = GuiCmd_Eval(req.data.c_str());
    } else {
      if (req.msg == "start") {
        std::string cmd = "LoadTestRun " + default_testrun + "; StartSim";
        if (!req.data.empty()) cmd = req.data + "; StartSim";
        resp.res = GuiCmd_Eval(cmd.c_str());
      } else if (req.msg == "stop")
        resp.res = GuiCmd_Eval("StopSim");
    }
  } else if (req.type == "cmd") {
    /* Commands directly to CarMaker executable
     * Warning: Information normally provided by CarMaker GUI might be missing
     */
    if (req.msg == "start") {
      std::string testrun = req.data.empty() ? default_testrun : req.data;
      // Most strings are already provided by CarMaker GUI
      SimStart(nullptr, ros::this_node::getName().c_str(), testrun.c_str(),
               nullptr, nullptr);
    } else if (req.msg == "stop") {
      SimStop2(0);
    }
    resp.res = 0;
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
    Log::printLog("  -> Node Name = " + ros::this_node::getName());
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

  ros::V_string topics;
  ros::this_node::getAdvertisedTopics(topics);

  {
    Log::printLog("  -> Advertised Topics (" + std::to_string(topics.size()) +
                  "):");
    for (auto const& name : topics) {
      Log::printLog("    -> " + name);
    }
    topics.clear();
    ros::this_node::getSubscribedTopics(topics);
    Log::printLog("  -> Subscribed Topics (" + std::to_string(topics.size()) +
                  "):");
    for (auto const& name : topics) {
      Log::printLog("    -> " + name);
    }
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
    ros::spinOnce();
    return;
  }

  scheduler_.executeJobs(CMJob::CallbackHook::PreIn, rel_cycle_num_);

  if (sync_mode_ == SyncMode::kTopic) {
    scheduler_.lock(rel_cycle_num_, ros::spinOnce);
  } else {
    ros::spinOnce();
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

  if (ros::isInitialized()) {
    ros::shutdown();
  }
  if (nhp_) {
    nhp_->shutdown();
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
