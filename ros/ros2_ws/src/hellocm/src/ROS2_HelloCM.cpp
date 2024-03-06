/*!
 ******************************************************************************
 **  External ROS Node for simple communication with CarMaker
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#include <chrono>
#include <memory>

#include "hellocm_msgs/msg/cm2_ext.hpp"
#include "hellocm_msgs/msg/ext2_cm.hpp"
#include "hellocm_msgs/srv/init.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class HelloCM : public rclcpp::Node {
 public:
  HelloCM() : Node("hellocm"), cycle_no_(0), delay_(0) {
    RCLCPP_INFO(this->get_logger(), "%s -> Start spinning...",
                rclcpp::Node::get_name());
    init();
  }

  ~HelloCM() {
    RCLCPP_INFO(this->get_logger(), "%s -> Shutdown", rclcpp::Node::get_name());
  }

 private:
  /*! Cyclic log dependent on wall time (system time) */
  void on_wall_timer() {
    RCLCPP_INFO(this->get_logger(), "%s: I am Alive!", this->get_name());
  }

  /*! Callback function for timer dependent on simulation time */
  void on_timer();

  /*! Subscription callback function */
  void topic_callback(const hellocm_msgs::msg::CM2Ext::SharedPtr msg);

  /*! Service callback function */
  void handle_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hellocm_msgs::srv::Init::Request> request,
      const std::shared_ptr<hellocm_msgs::srv::Init::Response> response);

  /*! Set up subscriber, publisher and timers */
  void init();

  /*!< Current cycle number of this node */
  size_t cycle_no_;

  /*!< Expected cycle time in milliseconds of external ROS Node.
   *    - Might be used for synchronization (under development)
   *    - e.g. usage in combination of published message
   */
  int cycle_time_;

  /*!< Delay in seconds to demonstrate synchronization mechanism */
  double delay_;

  rclcpp::Publisher<hellocm_msgs::msg::Ext2CM>::SharedPtr publisher_;
  rclcpp::Subscription<hellocm_msgs::msg::CM2Ext>::SharedPtr subscription_;

  /*!< Service for resetting this node e.g. when simulation starts */
  rclcpp::Service<hellocm_msgs::srv::Init>::SharedPtr service_;

  /*!< Timer independent from simulation time */
  rclcpp::TimerBase::SharedPtr wall_timer_;

  /*!< Timer dependent on simulation time ("/clock" topic) */
  rclcpp::TimerBase::SharedPtr timer_;
};

void HelloCM::init() {
  publisher_ = this->create_publisher<hellocm_msgs::msg::Ext2CM>("ext2cm", 10);
  subscription_ = this->create_subscription<hellocm_msgs::msg::CM2Ext>(
      "cm2ext", 10, std::bind(&HelloCM::topic_callback, this, _1));
  service_ = this->create_service<hellocm_msgs::srv::Init>(
      "init", std::bind(&HelloCM::handle_service, this, _1, _2, _3));

  bool use_sim_time;
  this->get_parameter("use_sim_time", use_sim_time);

  if (use_sim_time) {
    RCLCPP_INFO(this->get_logger(), "  -> Use time provided by Clock Server!");
  } else {
    RCLCPP_INFO(this->get_logger(), "  -> Use system time!");
  }

  const char param[] = "cycletime";
  this->declare_parameter<int>(param, 10000);
  this->get_parameter(param, cycle_time_);
  RCLCPP_INFO(this->get_logger(), "  -> Cycle time = %dms", cycle_time_);

  wall_timer_ =
      create_wall_timer(10s, std::bind(&HelloCM::on_wall_timer, this));
  timer_ = rclcpp::create_timer(this, this->get_clock(),
                                std::chrono::milliseconds(cycle_time_),
                                std::bind(&HelloCM::on_timer, this));

  // Print general information after everything is done
  RCLCPP_INFO(this->get_logger(), "%s", "Initialization of ROS Node finished!");
  RCLCPP_INFO(this->get_logger(), "  -> Node Name = '%s'", this->get_name());
  RCLCPP_INFO(this->get_logger(), "  -> Namespace = '%s'",
              this->get_namespace());

  // Advertised and subscribed Topics
  // TODO: Does this list node-specific topics or all?
  auto topics = this->get_topic_names_and_types();
  RCLCPP_INFO(this->get_logger(), "  -> Topics (%lu)", topics.size());
  for (const auto& topic : topics) {
    RCLCPP_INFO(this->get_logger(), "   -> %s (%s)", topic.first.c_str(),
                topic.second.data()->c_str());
  }

  auto services = this->get_service_names_and_types();
  RCLCPP_INFO(this->get_logger(), "  -> Services     (%lu)", services.size());
  for (const auto& topic : services) {
    RCLCPP_INFO(this->get_logger(), "   -> %s (%s)", topic.first.c_str(),
                topic.second.data()->c_str());
  }
}

void HelloCM::on_timer() {
  hellocm_msgs::msg::Ext2CM msg;
  msg.cycleno = static_cast<long>(++cycle_no_);
  msg.time = this->now();

  // delay node to demonstrate the effect of synchronization
  // TODO: mutex?
  if (delay_ > 1e-9) {
    std::chrono::duration<double> sleep_s(delay_);
    auto sleep_ms(
        std::chrono::duration_cast<std::chrono::milliseconds>(sleep_s));
    rclcpp::sleep_for(sleep_ms);
  }

  // set time stamp in header just before publishing
  msg.header.stamp = rclcpp::Clock().now();

  // publish the message
  publisher_->publish(msg);

  RCLCPP_INFO(this->get_logger(),
              "%s - PubTimer (ROS Time=%.3fs): This Node is in cycle %lu, "
              "Time=%.3fs, Stamp=%.3fs",
              this->get_name(), rclcpp::Clock().now().seconds(), msg.cycleno,
              rclcpp::Time(msg.time).seconds(),
              rclcpp::Time(msg.header.stamp).seconds());
}

void HelloCM::topic_callback(const hellocm_msgs::msg::CM2Ext::SharedPtr msg) {
  // Update variables
  // TODO: mutex?
  delay_ = msg->synthdelay;
  RCLCPP_INFO(this->get_logger(),
              "%s - TpcIn (ROS Time=%.3fs): CarMaker Node is in cycle %lu, "
              "Time=%.3fs, Stamp=%.3fs",
              this->get_name(), this->now().seconds(), msg->cycleno,
              rclcpp::Time(msg->time).seconds(),
              rclcpp::Time(msg->header.stamp).seconds());
}

void HelloCM::handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<hellocm_msgs::srv::Init::Request> request,
    const std::shared_ptr<hellocm_msgs::srv::Init::Response> response) {
  (void)request_header;
  (void)request;

  RCLCPP_INFO(this->get_logger(), "%s: Service 'Init' was triggered",
              this->get_name());

  // reset variables
  cycle_no_ = 0;
  delay_ = 0.0;

  /*
   * Check if parameter has changed!
   *  TODO: use topic /parameter_events instead?
   */
  const char param[] = "cycletime";

  if (this->has_parameter(param)) {
    int value;
    this->get_parameter(param, value);

    if (cycle_time_ != value) {
      cycle_time_ = value;
      RCLCPP_INFO(this->get_logger(), "  -> Update param '%s' to '%d'", param,
                  cycle_time_);

      timer_ = create_timer(this, this->get_clock(),
                            std::chrono::milliseconds(cycle_time_),
                            std::bind(&HelloCM::on_timer, this));
    }
  }

  response->success = 1;
  response->cycletime = cycle_time_;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloCM>());
  rclcpp::shutdown();
  return 0;
}
