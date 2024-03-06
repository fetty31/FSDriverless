/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef PARAMS_CMNODE_HPP
#define PARAMS_CMNODE_HPP

#include <vector>
#include <cmath>
#include <string>
#include <ros/ros.h>

typedef struct tCameraRSI {
    char*           name;
    double*         pos;                /*!< Mounting position on vehicle frame */
    double*         rot;                /*!< Mounting rotation on vehicle frame */
    double          FoV;
    double*         Resolution;         /*!< Resolution width and height */
    int             UpdRate;
    int             nCycleOffset;
    int             Socket;
} tCameraRSI;

namespace cm_yaml{

/* Lidar RSI --> Number of Beams */
static const unsigned int tableSize = 15000 * 6;

static struct {
    bool                use_vc;
    bool                use_llc;
    bool                use_4wd;
    int                 selector_ctrl;
    double              gas;
    double              brake;
    double              steer_ang;
    double              steer_ang_vel;
    double              steer_ang_acc;
} VC; /* Vehicle Controls */

static struct {
    bool receiving = false; // this param exists to ensure the trq/vel control is computed at the receiving frequency
    double target_trq[4];
    double target_vel[4];
    double ratio = 9.6; // transmission ratio
} PTC; /* PowerTrain Control*/

struct {
    int nSensors;
    struct {
        int             Active;             /*!< Presence of active Object sensors */
        double*         pos;                /*!< Mounting position on vehicle frame */
        double*         rot;                /*!< Mounting rotation on vehicle frame */
        int             UpdRate;
        int             nCycleOffset;
    } Object;

    struct {
        int             Active;             /*!< Presence of active Camera sensors */
        double*         pos;                /*!< Mounting position on vehicle frame */
        double*         rot;                /*!< Mounting rotation on vehicle frame */
        int             UpdRate;
        int             nCycleOffset;
    } Camera;

    struct {
        int             Active;                 /*!< Presence of active LidarRSI sensors */
        double*         pos;                    /*!< Mounting position on vehicle frame */
        double*         rot;                    /*!< Mounting rotation on vehicle frame */
        int             UpdRate;
        int             nCycleOffset;
        std::string     frame_id = "lidar";

        /* Variables to be used in LidarRSI processing. */
        double *FOV_h;
        double *FOV_v;
        double nBeams_h;
        double nBeams_v;
        double BeamTable[tableSize];
        int ret, rows = 0;
        double x[tableSize / 6];
        double y[tableSize / 6];
        double z[tableSize / 6];
        double* nBeams;
        int cycleCounter = 0;
    } LidarRSI;

    std::vector<tCameraRSI> CameraRSI;

} Sensor; /*!< Sensor parameters */

struct Params{

    // Constructor
    Params(const ros::NodeHandlePtr nh);

    struct Topics{

        // AS commands
        std::string commands;
        std::string steering;

        // Finish
        std::string finish;

        // AS state
        std::string carstate;
        std::string lidar;

        // LLC final commands
        std::string motorFL;
        std::string motorFR;
        std::string motorRL;
        std::string motorRR;

        // Sensors:
            // Inverters
        std::string inverterFL;
        std::string inverterFR;
        std::string inverterRL;
        std::string inverterRR;

            // Vectornav
        std::string odom;
        std::string imu;
        std::string pose;

            // Battery
        std::string battery;

            // SBGs
        std::string sbg_front;
        std::string sbg_rear;

            // Actuators
        std::string steer_sensor;

            // Pressure
        std::string pressure;

            // Pedals
        std::string throttle;
        std::string brake;

        // Estimation
        std::string kinematic;
        std::string dynamic;

    } topics;

    struct CycleTimes{

        // AS commands
        int commands;
        int steering;

        // Finish
        int finish;

        // AS state
        int carstate;
        int lidar;

        // LLC final commands
        int motorFL;
        int motorFR;
        int motorRL;
        int motorRR;

        // Sensors:
            // Inverters
        int inverterFL;
        int inverterFR;
        int inverterRL;
        int inverterRR;

            // Vectornav
        int odom;
        int imu;
        int pose;

            // Battery
        int battery;

            // SBGs
        int sbg_front;
        int sbg_rear;

            // Pressure
        int pressure;

            // Pedals
        int throttle;
        int brake;

            // Actuators
        int steer_sensor;

        // Estimation
        int kinematic;
        int dynamic;
        
    } hz;

    struct Config{
        bool lateralCtrl;       // Lateral control pipeline flag
        bool driver;            // IPG driver flag
        bool hidraulic_brake;   // Hidraulic Brake flag
        bool EBS;               // EBS (only in finish) flag
        int LLC;                // Low Level Control flag
        bool wd4;               // 4 Wheel Drive flag
    } config;

    struct Steering{
        double maxAtWheel;  // max/min steering angle in wheel frame (not steering wheel) [rad]
        double minAtWheel;
    } steering;

    struct Noise{
        double mean_odom;
        double std_odom;
        double mean_imu;
        double std_imu;
    } noise;
};

}

#endif