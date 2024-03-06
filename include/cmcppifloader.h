/*!
 ******************************************************************************
 **  CarMaker C++ Interface Loader module
 **  Unsupported example provided by IPG free of charge
 **  The structure may change in the future
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 */

#ifndef CMCPPIFLOADER_H
#define CMCPPIFLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CMCppIFLoader_load loads shared library of given interface and
 * parameters from its respective Infofile. This function is supposed to be
 * called in User.c->User_Init() for each interface.
 * This module expects to find a shared library named \a 'lib<cmif_name>.so' and
 * an Infofile named \a 'Data/Config/<cmif_name>Parameters'.
 * If the Infofile Key \a Cfg.Mode is set to 0, or if the major version number
 * of the loaded interface doesn't match with the loader module, the interface
 * will be disabled.
 * @param cmif_name The name of the interface, e.g. "CMRosIF"
 */
void CMCppIFLoader_load(const char* const cmif_name);

/**
 * @brief CMCppIFLoader_getSymbol to retrieve additional custom function pointer
 * pointers from previously loaded interface. This usage of this function is
 * optional and it is supposed to be called in User.c->User_Init() right after
 * CMCppIFLoader_load().
 * @pre call to CMCppIFLoader_load() with the name of the interface
 * @param symbol name of requested symbol, e.g. "CMNodeHelloCM_customFunction"
 * @return Function pointer to requested symbol
 */
void* CMCppIFLoader_getSymbol(const char* symbol);

/**
 * @brief CMCppIFLoader_getInterfacePtr to retrieve pointer to instantiated
 * interface from previously loaded interface. This usage of this function is
 * optional and it is supposed to be called in User.c->User_Init() right after
 * CMCppIFLoader_load().
 * @pre call to CMCppIFLoader_load() with the name of the interface
 * @return Pointer to instantiated interface class which is needed for calling
 * custom functions retrieved by CMCppIFLoader_getSymbol.
 */
void* CMCppIFLoader_getInterfacePtr();

/**
 * @brief CMCppIFLoader_init calls the hook function
 * CarMakerCPPInterface::init() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_init(void);

/**
 * @brief CMCppIFLoader_declQuants calls the hook function
 * CarMakerCPPInterface::declQuants() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 */
void CMCppIFLoader_declQuants(void);

/**
 * @brief CMCppIFLoader_testrunStartAtBegin calls the hook function
 * CarMakerCPPInterface::testrunStartAtBegin() of each loaded interface and
 * provides it with a pointer to the appropriate Infofile handle. If any
 * associated Infofile has changed since the last simulation start, it will be
 * reloaded.
 * @pre call to CMCppIFLoader_load() must have been called before
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_testrunStartAtBegin(void);

/**
 * @brief CMCppIFLoader_testrunStartAtEnd calls the hook function
 * CarMakerCPPInterface::testrunStartAtEnd() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_testrunStartAtEnd(void);

/**
 * @brief CMCppIFLoader_testrunRampUp calls the hook function
 * CarMakerCPPInterface::testrunRampUp() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @return 1 if all interfaces are ready, otherwise 0
 */
int CMCppIFLoader_testrunRampUp(void);

/**
 * @brief CMCppIFLoader_testrunEnd calls the hook function
 * CarMakerCPPInterface::testrunEnd() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_testrunEnd(void);

/**
 * @brief CMCppIFLoader_in calls the hook function CarMakerCPPInterface::in()
 * of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 */
void CMCppIFLoader_in(void);

/**
 * @brief CMCppIFLoader_drivmanCalc calls the hook function
 * CarMakerCPPInterface::drivmanCalc() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @param dt the simulation time step
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_drivmanCalc(const double* const dt);

/**
 * @brief CMCppIFLoader_vehicleControlCalc calls the hook function
 * CarMakerCPPInterface::vehicleControlCalc() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @param dt the simulation time step
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_vehicleControlCalc(const double* const dt);

/**
 * @brief CMCppIFLoader_calc calls the hook function
 * CarMakerCPPInterface::calc() of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @param dt the simulation time step
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_calc(const double* const dt);

/**
 * @brief CMCppIFLoader_out calls the hook function CarMakerCPPInterface::out()
 * of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 */
void CMCppIFLoader_out(void);

/**
 * @brief CMCppIFLoader_end calls the hook function CarMakerCPPInterface::end()
 * of each loaded interface
 * @pre call to CMCppIFLoader_load() must have been called before
 * @return the lowest return value obtained by the hook function of the loaded
 * interfaces
 */
int CMCppIFLoader_end(void);

#ifdef __cplusplus
}
#endif

#endif  // CMCPPIFLOADER_H
