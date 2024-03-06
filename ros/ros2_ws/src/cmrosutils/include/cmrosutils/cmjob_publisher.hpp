#pragma once

#include <functional>

#include "CMJob.h"
#include "rclcpp/rclcpp.hpp"

namespace CMJob {

/*!
 * \brief Ros_Publisher provides a job based interfoce for ros publisher
 */
template <typename T>
class RosPublisher final : public AbstractJob {
 public:
  RosPublisher<T>(const rclcpp::Node::SharedPtr& node, const std::string& topic,
                  const rclcpp::QoS& qos = rclcpp::QoS(1))
      : AbstractJob(topic, JobType::Cyclic, false), node_(node), topic_(topic) {
    msg_ = T();
    pub_ = node_->template create_publisher<T>(topic_, qos);
    this->setCallbackHook(CallbackHook::Out);
  }

  /*!
   * \brief Execution of the job
   */
  void execute(void);

  /*!
   * \brief register_callback
   * \param fp function pointer to callback function
   */
  void registerCallback(std::function<void(T&)> fp) {
    user_callback_ = std::bind(fp, std::placeholders::_1);
  }

  /*!
   * \brief register_callback
   * \param fp function pointer to callback function
   * \param obj pointer to the object (e.g. this)
   */

  template <typename M>
  void registerCallback(void (M::*fp)(T&), M* obj) {
    user_callback_ = std::bind(fp, obj, std::placeholders::_1);
  }

 private:
  rclcpp::Node::SharedPtr node_ = nullptr; /*! ros node handle */

  std::string topic_;                            /*! ros topic name */
  typename rclcpp::Publisher<T>::SharedPtr pub_; /*! ros publisher */

  std::function<void(T&)> user_callback_; /*! callback function */
  T msg_;                                 /*! message buffer */
};

///////////////////////////////////////////////////////////////////////////////
//                               IMPLEMENTATION                              //
///////////////////////////////////////////////////////////////////////////////

template <typename T>
void RosPublisher<T>::execute(void) {
  if (this->user_callback_) {
    this->user_callback_(this->msg_);
    this->pub_->publish(this->msg_);
  }

  AbstractJob::execute();
}

}  // namespace CMJob
