#pragma once

#include <functional>

#include "CMJob.h"
#include "ros/ros.h"

namespace CMJob {

/*!
 * \brief Ros_Subscriber provides a job based interfoce for ros subscribers
 */

template <typename T>
class RosSubscriber final : public AbstractJob {
 public:
  RosSubscriber<T>(const JobType& type, const bool& sync,
                   const ros::NodeHandlePtr& node, const std::string& topic,
                   const size_t& queue_size = 1)
      : AbstractJob(topic, type, sync),
        node_(node),
        topic_(topic),
        queue_size_(queue_size) {}

  /*!
   * \brief Initialization of the job
   */
  void init(void);

  /*!
   * \brief Execution of the job
   */
  void execute(void);

  /*!
   * \brief register_callback
   * \param fp function pointer to callback function
   */
  void registerCallback(std::function<void(const T&)> fp) {
    user_callback_ = std::bind(fp, std::placeholders::_1);
  }

  /*!
   * \brief register_callback
   * \param fp function pointer to callback function
   * \param obj pointer to the object (e.g. this)
   */

  template <typename M>
  void registerCallback(void (M::*fp)(const T&), M* obj) {
    user_callback_ = std::bind(fp, obj, std::placeholders::_1);
  }

  /*!
   * \brief callback function, which is used for the ros subscriber
   * \param[in] msg
   */
  void rosCallback(const T& msg);

 private:
  ros::NodeHandlePtr node_ = nullptr; /*! ros node handle */

  std::string topic_;     /*! ros topic name */
  ros::Subscriber sub_;   /*! ros subscriber */
  size_t queue_size_ = 1; /*! ros queue size */

  std::function<void(const T&)> user_callback_; /*! callback function */
  T msg_;                                       /*! message buffer */
};

///////////////////////////////////////////////////////////////////////////////
//                               IMPLEMENTATION                              //
///////////////////////////////////////////////////////////////////////////////

template <typename T>
void RosSubscriber<T>::init(void) {
  /* add ros subscriber */
  this->sub_ = this->node_->subscribe(this->topic_, this->queue_size_,
                                      &RosSubscriber<T>::rosCallback, this);

  AbstractJob::init();
}

template <typename T>
void RosSubscriber<T>::execute(void) {
  if (this->user_callback_ && this->msg_ != nullptr) {
    this->user_callback_(this->msg_);
  }

  AbstractJob::execute();
}

template <typename T>
void RosSubscriber<T>::rosCallback(const T& msg) {
  if (this->getJobState() == JobState::Disabled) return;

  this->msg_ = msg;
  prepare();

  if (this->getCallbackHook() == CallbackHook::Manual) {
    execute();
  }
}

}  // namespace CMJob
