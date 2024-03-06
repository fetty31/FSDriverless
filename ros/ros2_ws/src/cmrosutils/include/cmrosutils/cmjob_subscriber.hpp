#pragma once

#include <functional>

#include "CMJob.h"
#include "rclcpp/rclcpp.hpp"

namespace CMJob {

/*!
 * \brief Ros_Subscriber provides a job based interfoce for ros subscribers
 */

template <typename T>
class RosSubscriber final : public AbstractJob {
 public:
  RosSubscriber<T>(const JobType& type, const bool& sync,
                   const rclcpp::Node::SharedPtr& node,
                   const std::string& topic,
                   const rclcpp::QoS& qos = rclcpp::QoS(1))
      : AbstractJob(topic, type, sync), node_(node) {
    sub_ = node_->template create_subscription<T>(
        topic, qos,
        std::bind(&RosSubscriber::rosCallback, this, std::placeholders::_1));
  }

  /*!
   * \brief Execution of the job
   */
  void execute(void);

  /*!
   * \brief register_callback
   * \param fp function pointer to callback function
   */
  void registerCallback(std::function<void(typename T::ConstSharedPtr)> fp) {
    user_callback_ = std::bind(fp, std::placeholders::_1);
  }

  /*!
   * \brief register_callback
   * \param fp function pointer to callback function
   * \param obj pointer to the object (e.g. this)
   */

  template <typename M>
  void registerCallback(void (M::*fp)(typename T::ConstSharedPtr), M* obj) {
    user_callback_ = std::bind(fp, obj, std::placeholders::_1);
  }

  /*!
   * \brief callback function, which is used for the ros subscriber
   * \param[in] msg
   */
  void rosCallback(typename T::ConstSharedPtr msg);

 private:
  rclcpp::Node::SharedPtr node_ = nullptr; /*! ros node handle */

  typename rclcpp::Subscription<T>::SharedPtr sub_; /*! ros subscriber */

  /*! callback function */
  std::function<void(typename T::ConstSharedPtr)> user_callback_;

  typename T::SharedPtr msg_; /*! message buffer */
};

///////////////////////////////////////////////////////////////////////////////
//                               IMPLEMENTATION                              //
///////////////////////////////////////////////////////////////////////////////

template <typename T>
void RosSubscriber<T>::execute(void) {
  if (user_callback_ && msg_) {
    user_callback_(msg_);
  }

  AbstractJob::execute();
}

template <typename T>
void RosSubscriber<T>::rosCallback(typename T::ConstSharedPtr msg) {
  if (this->getJobState() == JobState::Disabled) return;

  msg_ = std::make_shared<T>(*msg);
  prepare();

  if (getCallbackHook() == CallbackHook::Manual) {
    execute();
  }
}

}  // namespace CMJob
