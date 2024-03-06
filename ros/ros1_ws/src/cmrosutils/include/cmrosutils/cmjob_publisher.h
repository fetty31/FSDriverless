#pragma once

#include <functional>

#include "CMJob.h"
#include "ros/ros.h"

namespace CMJob {

/*!
 * \brief Ros_Publisher provides a job based interfoce for ros publisher
 */
template <typename T>
class RosPublisher final : public AbstractJob {
 public:
  RosPublisher<T>(const ros::NodeHandlePtr& node, const std::string& topic,
                  const size_t& queue_size = 1)
      : AbstractJob(topic, JobType::Cyclic, false),
        node_(node),
        topic_(topic),
        queue_size_(queue_size) {
    initMsg(tag<T>{});
    initPublisher(tag<T>{});

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
  ros::NodeHandlePtr node_ = nullptr; /*! ros node handle */

  std::string topic_;     /*! ros topic name */
  ros::Publisher pub_;    /*! ros publisher */
  size_t queue_size_ = 1; /*! ros queue size */

  std::function<void(T&)> user_callback_; /*! callback function */
  T msg_;                                 /*! message buffer */

  /* Partial template specialization */
  template <typename>
  struct tag {};

  template <class R>
  void initPublisher(tag<R>) {
    this->pub_ =
        this->node_->template advertise<R>(this->topic_, this->queue_size_);
  }

  template <class R>
  void initPublisher(tag<boost::shared_ptr<R>>) {
    this->pub_ =
        this->node_->template advertise<R>(this->topic_, this->queue_size_);
  }

  template <class R>
  void initMsg(tag<R>) {}

  template <class R>
  void initMsg(tag<boost::shared_ptr<R>>) {
    this->msg_ = boost::make_shared<R>();
  }
};

///////////////////////////////////////////////////////////////////////////////
//                               IMPLEMENTATION                              //
///////////////////////////////////////////////////////////////////////////////

template <typename T>
void RosPublisher<T>::execute(void) {
  if (this->user_callback_ && this->msg_ != nullptr) {
    this->user_callback_(this->msg_);
    this->pub_.publish(this->msg_);
  }

  AbstractJob::execute();
}

}  // namespace CMJob
