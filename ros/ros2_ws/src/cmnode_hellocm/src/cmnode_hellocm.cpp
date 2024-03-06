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

#include "cmnode_hellocm.hpp"

using cm_ros::CMNodeHelloCM;
using CMJob::Log;

CMNodeHelloCM::CMNodeHelloCM() {}

void CMNodeHelloCM::ext2cmCallback(
    hellocm_msgs::msg::Ext2CM::ConstSharedPtr msg) {
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(3);
  ss << "\t[" << rclcpp::Clock().now().seconds();
  if (nhp_->get_clock()->ros_time_is_active()) {
    ss << ", " << nhp_->now().seconds();
  }
  ss << "]: " << nhp_->get_fully_qualified_name() << ": Sub Msg: ";
  ss << "Time " << rclcpp::Time(msg->time).seconds() << "s, ";
  ss << "Cycle " << msg->cycleno;

  Log::printLog(ss.str());
}

void CMNodeHelloCM::cm2extFillMsg(hellocm_msgs::msg::CM2Ext& msg) {
  msg.cycleno = static_cast<uint32_t>(rel_cycle_num_);
  msg.time =
      rclcpp::Time(static_cast<int64_t>(1e9 * SimCore.Time), RCL_ROS_TIME);
  msg.synthdelay = synth_delay_;
  msg.header.stamp = rclcpp::Clock().now();
}

int CMNodeHelloCM::userInit() {
  synth_delay_ = 1e-6;
  {  // set up cm2ext publisher + job
    typedef CMJob::RosPublisher<hellocm_msgs::msg::CM2Ext> cm2ext_t;
    auto job = std::make_shared<cm2ext_t>(nhp_, "cm2ext");
    job->setCycleTime(15000);
    job->registerCallback(&CMNodeHelloCM::cm2extFillMsg, this);
    scheduler_.addJob(job);
  }

  Log::printLog("  -> Initializing service client /hellocm/init");
  srv_init_ = nhp_->create_client<hellocm_msgs::srv::Init>("/hellocm/init");
  param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(nhp_, "/hellocm/hellocm");

  return 1;
}

void CMNodeHelloCM::userDeclQuants() {
  tDDefault* df = DDefaultCreate("CMRosIF.HelloCM.");
  DDefULong(df, "CycleNoRel", "ms", &rel_cycle_num_, DVA_None);
  DDefDouble4(df, "SynthDelay", "s", &synth_delay_, DVA_IO_In);
  DDefaultDelete(df);
}

int CMNodeHelloCM::userTestrunStartAtBegin() {
  // Prepare external node for next simulation
  if (!srv_init_->wait_for_service(std::chrono::seconds(2))) {
    Log::printError(EC_Sim,
                    "ROS service is not ready! Please start external ROS node "
                    "providing service '" +
                        std::string(srv_init_->get_service_name()) + "'!");
    node_mode_ = NodeMode::kDisabled;
    return -1;
  }

  Log::printLog("  -> Sending service request");
  srv_init_->async_send_request(
      std::make_shared<hellocm_msgs::srv::Init::Request>());

  {  // set up ext2cm subscriber + job
    std::string topic = "ext2cm";
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    auto cycle_time = param_client_->get_parameter("cycletime", 15000);

    typedef CMJob::RosSubscriber<hellocm_msgs::msg::Ext2CM> ext2cm_t;
    auto job = std::make_shared<ext2cm_t>(job_type, synchronize, nhp_, topic);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(&CMNodeHelloCM::ext2cmCallback, this);
    scheduler_.addJob(job);

    if (cycle_time % clock_cycle_time_ != 0 ||
        (cycle_time < clock_cycle_time_ && clock_cycle_time_ > 0)) {
      node_mode_ = NodeMode::kDisabled;
      LogErrF(EC_Sim,
              "Ext. ROS node has an invalid cycle time! Expected multiple of "
              "%iums but got %ims",
              clock_cycle_time_, cycle_time);

      return -1;
    }
  }

  return 1;
}

int CMNodeHelloCM::userTestrunEnd() {
  scheduler_.deleteJob("ext2cm");
  return 1;
}

int CMNodeHelloCM::customFunction(const int input, char* output) {
  strcpy(output, __PRETTY_FUNCTION__);
  return clock_cycle_time_ + input;
}

int CMNodeHelloCM_customFunction(void* intf, const int input, char* output) {
  return reinterpret_cast<CMNodeHelloCM*>(intf)->customFunction(input, output);
}

// Important: Use this macro to register the derived class as an interface with
// the CarMaker C++ Interface Loader module
REGISTER_CARMAKER_CPP_IF(CMNodeHelloCM)
