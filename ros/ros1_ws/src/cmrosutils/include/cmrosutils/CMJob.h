/*!
 * @file CMJob.h
 * @author Fabian Häußler
 *
 * @brief Library for providing a generic interface to handle jobs
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace CMJob {

/*!
 *  \addtogroup CMJob
 *  @{
 */

/*!
 * \enum JobType
 *
 * These are the available job types
 *
 * \anchor JobType
 */
enum class JobType {
  Cyclic,  ///< called periodically at the end of the job
  Trigger  ///< called immediately
};

/*!
 * \enum JobState
 *
 * These are the states of the statemachine
 *
 * \anchor JobState
 */
enum class JobState { New, Active, Ready, Idle, Suspended, Timeout, Disabled };

/*!
 * \enum CallbackHook
 *
 * The user can add additional enums after the last one in the list.
 * The one has to use type casts.
 *
 * \anchor CallbackHook
 */
enum class CallbackHook {
  Manual = -1,
  PreIn,
  In,
  DrivMan,
  Traffic,
  VehicleControl,
  Brake,
  Calc,
  Out,
  User_1 = 32,
  User_2,
  User_3,
  User_4,
  User_5
};
/*! @} */

/*!
 * \brief The AbstractJob class provides an interface for jobs
 */

class AbstractJob {
 public:
  /*!
   * \brief AbstractJob
   * \param name is the job name
   * \param type is the type Cyclic or Trigger
   * \param sync is the flag if the job should get syncronized
   */
  AbstractJob(const std::string& name, JobType type, bool sync);
  ~AbstractJob();

  /* Configuration */
  /* Can only be set, when the job is in state JobState::New */
  /*!
   * \brief setCycleTime
   * \param cycle
   */
  void setCycleTime(unsigned long cycle);
  /*!
   * \brief setCycleOffset
   * \param cycle
   */
  void setCycleOffset(unsigned long cycle);
  /*!
   * \brief setTimeoutTime
   * \param time_s
   */
  void setTimeoutTime(double time_s);
  /*!
   * \brief setExecutionCounter
   * \param count
   */
  void setExecutionCounter(int count);
  /*!
   * \brief skipFirstNCycles
   * \param count
   */
  void skipFirstCycles(unsigned count);
  /*!
   * \brief setCallbackHook
   * \param hook
   */
  void setCallbackHook(CallbackHook hook);

  /* State machine */

  /*!
   * \brief Initialization of the job
   *
   * Function init() **can** be provided in the inherited class.
   * Then AbstractJob::init() has to be called.
   *
   * Use it to
   *   - set default job settings
   *   - external initialization
   */
  virtual void init();

  /*!
   * \brief Activation of the job
   *
   * Function activate() **can** be provided in the inherited class.
   * Then AbstractJob::activate() has to be called.
   *
   * Use it to
   *   - activate an idle job
   *   - reset variables
   */
  virtual void activate();

  /*!
   * \brief Preperation of the job
   *
   * Function prepare() **can** be provided in the inherited class.
   * Then AbstractJob::prepare() has to be called.
   *
   * Useful when you use a cyclic job, because then the execution is scheduled
   * and uses data provided by this function
   *
   * Use it to
   *   - update variables used for execution
   */
  virtual void prepare();

  /*!
   * \brief Execution of the job
   *
   * Function execute() **has** to be provided in the inherited class.
   * Then AbstractJob::execute() has to be called.
   *
   * Use it to
   *   - call the job functionality
   */
  virtual void execute() = 0;

  /*!
   * \brief Suspension of the job
   *
   * Function suspend() **can** be provided in the inherited class.
   * Then AbstractJob::suspend() has to be called.
   *
   * Use it to
   *   - suspend a job, which doesn't meet certain criteria
   */
  virtual void suspend();

  /*!
   * \brief Putting job into timeout
   *
   * Function timeout() **can** be provided in the inherited class.
   * Then AbstractJob::timeout() has to be called.
   */
  virtual void timeout();

  /*!
   * \brief Disable the job
   *
   * Function disable() **can** be provided in the inherited class.
   * Then AbstractJob::disable() has to be called.
   *
   * Use it to
   *   - disable the job
   */
  virtual void disable();

  /*!
   * \brief Reset the job
   *
   * Function reset() **can** be provided in the inherited class.
   * Then AbstractJob::reset() has to be called.
   *
   * Use it to
   *   - reset the job ( AbstractJob::init() gets called )
   */
  virtual void reset();

  /* Job functions */

  /*!
   * \brief Prints job details
   *
   * Prints \p name, \p cycletime and \p cycleoffset
   */
  void info() const;

  /*!
   * \brief function, which returns if the job is scheduled at \p cycle
   * \param[in] cycle
   * \return state if job is scheduled
   */

  bool isJobScheduled(unsigned long cycle) const;

  /*!
   * \brief function to check if current callback is scheduled at \p cycle
   * \param[in] cycle
   * \return state if callback is scheduled
   */
  bool isCallbackScheduled(unsigned long cycle) const;

  /*!
   * \brief isJobTimeout
   * \return state if job is in timeout
   */
  bool isJobTimeout(void) const;

  /* Getter */
  std::string getJobName() const;
  unsigned long getCycleTime() const;
  unsigned long getCycleOffset() const;
  bool isSynchronized() const;
  unsigned long getCycle() const;

  JobType getJobType() const;
  std::string getJobTypeName() const;

  CallbackHook getCallbackHook() const;
  std::string getCallbackHookName() const;

  JobState getJobState() const;
  std::string getJobStateName() const;

  /* Setter */
  void setCycle(unsigned long cycle);

  /*!
   * \brief set a job name instead of the argument in constructor
   *
   * with the same argument in the constructor, there can be multiple jobs
   * with the same name.
   */
  void setJobName(const std::string& name);

 protected:
  /* State setters */
  void setReady();
  void setIdle();
  void setActive();
  void setSuspended();
  void setTimeout();
  void setDisabled();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/*!
 * \brief The singleton JobScheduler class handles \p AbstractJobs
 */

class JobScheduler {
 public:
  /*!
   * \brief instance singleton
   * \return singleton instance of the class JobScheduler
   */
  static JobScheduler& instance() {
    static JobScheduler instance_;
    return instance_;
  }

  /*!
   * \brief SetCycleStep sets the cycle step, in which the scheduler should
   * operate \param cycle_step (default = 1ms)
   */

  void setCycleStep(unsigned short cycle_step);

  /*!
   * \brief Lock locks the simulation, when a job is suspended and releases
   * it, when this job gets prepared() within \p fp
   * \param[in] cycle
   * \param[in] fp function pointer, which prepares the suspended job to be
   * ready
   */
  void lock(unsigned long cycle, const std::function<void(void)>& fp);

  /*!
   * \brief AddJob adds the job to the container to handle it
   * \param job has to be a derived class of AbstractJob
   */

  void addJob(const std::shared_ptr<AbstractJob>& job);

  /*!
   * \brief AreJobsReady iterates through all jobs and checks if they are ready
   * \return state, if all jobs are ready at current timestamp
   * \param[in] cycle
   */
  bool areJobsReady(unsigned long cycle) const;

  /*!
   * \brief AreJobsTimedout iterates through all jobs and checks if they are
   * timedout \return state, if any job is in overtime
   */
  bool areJobsTimeout(void) const;

  /*!
   * \brief ExecuteJobs executes jobs at the defined hook point
   * \param[in] hook
   * \param[in] cycle
   */

  void executeJobs(CallbackHook hook, unsigned long cycle);

  /*!
   * \brief UpdateJobs iterates through all jobs and only sets individual jobs
   * active, when the job cycle is met. Also updates every job.
   * \param[in] cycle_current
   */
  void updateJobs(unsigned long cycle_current);

  /*!
   * \brief ResetJobs iterates through all jobs and sets them all to active
   */
  void resetJobs(void);

  /*!
   * \brief DisableJobs iterates through all jobs and sets them all to disabled
   */
  void disableJobs(void);

  /*!
   * \brief GetJob returns the job with \p name
   * \param name of the job
   * \return reference to the job
   */
  std::shared_ptr<CMJob::AbstractJob> getJob(const std::string& name) const;

  /*!
   * \brief DeleteJobs removes all jobs from the scheduler
   */
  void deleteJobs(void);

  /*!
   * \brief DeleteJob remove a job from the scheduler
   * \param[in] name of the job
   */
  void deleteJob(const std::string& name);

  /*!
   * \brief PrintJobs prints all jobs from the scheduler
   */
  void printJobs(void) const;

 private:
  JobScheduler() = default;
  JobScheduler(const JobScheduler&);
  JobScheduler& operator=(const JobScheduler&);

  /*! all jobs are registered here */
  std::vector<std::shared_ptr<AbstractJob>> Jobs_;

  unsigned short cycle_step_ = 1;
};

/*!
 * \brief The Log class
 *
 * This is a wrapper to register CarMaker Log functions.
 * If no CarMaker Log function is registered, it uses the std::cout output
 * stream
 */
class Log {
 public:
  /* set CM Log */
  static void setLog(void (*pLog)(const char* format, ...));
  static void setLogWarn(void (*pLogWarn)(unsigned ECId, const char* format,
                                          ...));
  static void setLogErr(void (*pLogErr)(unsigned ECId, const char* format,
                                        ...));

  /* Use these, if you want to print output depending on the registered output
   * stream */
  static void printLog(const std::string& out);
  static void printWarn(unsigned ECId, const std::string& out);
  static void printError(unsigned ECId, const std::string& out);

 private:
  static void (*cm_log)(const char* format, ...); /*! Log callback */
  static void (*cm_log_warn_f)(unsigned ECId, const char* format,
                               ...); /*! LogWarn callback */
  static void (*cm_log_err_f)(unsigned ECId, const char* format,
                              ...); /*! LogErr callback */
};

}  // namespace CMJob
