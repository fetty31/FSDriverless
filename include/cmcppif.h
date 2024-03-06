/*!
 ******************************************************************************
 **  CarMaker C++ Interface abstract class
 **  Unsupported example provided by IPG free of charge
 **  The structure may change in the future
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#ifndef CMCPPIF_H
#define CMCPPIF_H

#include <string>

// CarMaker header
#include "InfoUtils.h"

namespace cmcppif {

/**
 * @brief INTERFACE_VERSION Semantic version of this interface structure
 */
static const std::string INTERFACE_VERSION = "1.0.0";

/**
 * @brief The CarMakerCPPInterface abstract class provides the basic structure
 * for a C++ interface that can be dynamically loaded at runtime within the
 * CarMaker executable using the CarMaker C++ Interface Loader module. The
 * CarMakerCPPInterface class defines all hook functions needed by that module.
 */
class CarMakerCPPInterface {
 public:
  CarMakerCPPInterface() = default;

  virtual ~CarMakerCPPInterface() = default;

  /**
   * @brief getCMVersion is used to print the CarMaker version that the
   * interface was compiled for.
   * @return CarMaker version string e.g. "10.0.1"
   */
  virtual const char* getCMVersion() const = 0;

  /**
   * @brief getInterfaceVersion is used to check whether this interface is
   * compatible with the loader module.
   * @return Version string of this interface
   */
  virtual const std::string& getInterfaceVersion(void) const = 0;

  /**
   * @brief init basic initialization of the module. Called once at program
   * start. No realtime conditions.
   * @param infofile_ptr pointer to loaded infofile handle of the interface
   * @return < 0 if initialization fails and simulation should abort, >= 0
   * otherwise
   */
  virtual int init(struct tInfos* infofile_ptr) = 0;

  /**
   * @brief declQuants add user specific quantities to the dictionary. Called
   * once at program start. No realtime conditions.
   */
  virtual void declQuants(void) = 0;

  /**
   * @brief testrunStartAtBegin initialize and/or reset interface before
   * parameters are read in for a new simulation. Called when starting a new
   * TestRun in a separate thread (no realtime conditions).
   * @param infofile_ptr pointer to (re-)loaded infofile handle of the interface
   * @return < 0 if procedure fails and simulation should abort, >= 0 otherwise
   */
  virtual int testrunStartAtBegin(struct tInfos* infofile_ptr) = 0;

  /**
   * @brief testrunStartAtEnd initialize and/or reset interface after
   * parameters are read in for a new simulation. Called when starting a new
   * TestRun in a separate thread (no realtime conditions).
   * @return < 0 if procedure fails and simulation should abort, >= 0 otherwise
   */
  virtual int testrunStartAtEnd(void) = 0;

  /**
   * @brief testrunRampUp perform a smooth transition of variables. Hook is
   * called repeatedly until all interfaces and other modules return true.
   * @return false if interface is not ready, true if interface is ready
   */
  virtual int testrunRampUp(void) = 0;

  /**
   * @brief in called in the main loop in realtime context, at the very
   * beginning of the CarMaker cycle
   */
  virtual void in(void) = 0;

  /**
   * @brief drivmanCalc called in realtime context, after driving maneuver
   * calculation
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int drivmanCalc(const double& dt) = 0;

  /**
   * @brief vehicleControlCalc called in realtime context, after vehicle control
   * calculation
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int vehicleControlCalc(const double& dt) = 0;

  /**
   * @brief calc called in realtime context, after vehicle model has been
   * calculated
   * @param dt the simulation time step
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int calc(const double& dt) = 0;

  /**
   * @brief out called in the main loop in realtime context, close to end of
   * CarMaker cycle
   */
  virtual void out(void) = 0;

  /**
   * @brief testrunEnd to e.g. write data to file or clean up after the
   * simulation. Called in separate thread (no realtime conditions), when
   * TestRun ends.
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int testrunEnd(void) = 0;

  /**
   * @brief end to clean up before the application stops. Called at the end of
   * the program (no realtime conditions).
   * @return < 0 if errors occur, >= 0 otherwise
   */
  virtual int end(void) = 0;
};
}  // namespace cmcppif

using cmcppif_creator_t = cmcppif::CarMakerCPPInterface* (*)();

/**
 * @brief REGISTER_CARMAKER_CPP_IF macro to register a CarMaker CPP Interface
 * This macro is intended to be called at the end of the derived class
 * definition.
 * @param Derived The class definition derived from CarMakerCPPInterface
 * @return Instantiated object of the derived class
 */
#define REGISTER_CARMAKER_CPP_IF(Derived)              \
  extern "C" cmcppif::CarMakerCPPInterface* create() { \
    typedef Derived _derived;                          \
    return new _derived;                               \
  }

#endif  // CMCPPIF_H
