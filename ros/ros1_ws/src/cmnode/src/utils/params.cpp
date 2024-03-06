/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#include "utils/params.hh"

cm_yaml::Params::Params(const ros::NodeHandlePtr nh){

    // Subscribers
        // Cycle Times
    nh->param<int>("/IPG/Subscribers/Commands/Hz",  this->hz.commands,  10);
    nh->param<int>("/IPG/Subscribers/Steering/Hz",  this->hz.steering,  10);
    nh->param<int>("/IPG/Subscribers/Motors/FL/Hz", this->hz.motorFL,   10);
    nh->param<int>("/IPG/Subscribers/Motors/FR/Hz", this->hz.motorFR,   10);
    nh->param<int>("/IPG/Subscribers/Motors/RL/Hz", this->hz.motorRL,   10);
    nh->param<int>("/IPG/Subscribers/Motors/RR/Hz", this->hz.motorRR,   10);
    nh->param<int>("/IPG/Subscribers/Finish/Hz",    this->hz.finish,    10);
        // Topics
    nh->param<std::string>("/IPG/Subscribers/Commands/Topic",   this->topics.commands,   "/AS/C/commands");
    nh->param<std::string>("/IPG/Subscribers/Steering/Topic",   this->topics.steering,   "/AS/C/steering");
    nh->param<std::string>("/IPG/Subscribers/Motors/FL/Topic",  this->topics.motorFL,    "/CTRL/FrontLeftMotor");
    nh->param<std::string>("/IPG/Subscribers/Motors/FR/Topic",  this->topics.motorFR,    "/CTRL/FrontRightMotor");
    nh->param<std::string>("/IPG/Subscribers/Motors/RL/Topic",  this->topics.motorRL,    "/CTRL/RearLeftMotor");
    nh->param<std::string>("/IPG/Subscribers/Motors/RR/Topic",  this->topics.motorRR,    "/CTRL/RearRightMotor");
    nh->param<std::string>("/IPG/Subscribers/Finish/Topic",     this->topics.finish,     "/AS/C/finish");

    
    // Publishers
        // Cycle Times
    nh->param<int>("/IPG/Publishers/CarState/Hz",               this->hz.carstate,      10);
    nh->param<int>("/IPG/Publishers/Sensors/Odom/Hz",           this->hz.odom,          10);
    nh->param<int>("/IPG/Publishers/Sensors/Pose/Hz",           this->hz.pose,          10);
    nh->param<int>("/IPG/Publishers/Sensors/IMU/Hz",            this->hz.imu,           10);
    nh->param<int>("/IPG/Publishers/Sensors/SBG/Front/Hz",      this->hz.sbg_front,     10);
    nh->param<int>("/IPG/Publishers/Sensors/SBG/Rear/Hz",       this->hz.sbg_rear,      10);
    nh->param<int>("/IPG/Publishers/Sensors/Steering/Hz",       this->hz.steer_sensor,  10);
    nh->param<int>("/IPG/Publishers/Estimation/Kinematic/Hz",   this->hz.kinematic,     10);
    nh->param<int>("/IPG/Publishers/Estimation/Dynamic/Hz",     this->hz.dynamic,       10);
    nh->param<int>("/IPG/Publishers/Inverters/FL/Hz",           this->hz.inverterFL,    10);
    nh->param<int>("/IPG/Publishers/Inverters/FR/Hz",           this->hz.inverterFR,    10);
    nh->param<int>("/IPG/Publishers/Inverters/RL/Hz",           this->hz.inverterRL,    10);
    nh->param<int>("/IPG/Publishers/Inverters/RR/Hz",           this->hz.inverterRR,    10);
    nh->param<int>("/IPG/Publishers/Battery/Hz",                this->hz.battery,       10);
    nh->param<int>("/IPG/Publishers/Pressure/Hz",               this->hz.pressure,      10);
    nh->param<int>("/IPG/Publishers/Pedals/Throttle/Hz",        this->hz.throttle,      10);
    nh->param<int>("/IPG/Publishers/Pedals/Brake/Hz",           this->hz.brake,         10);
        // Topics
    nh->param<std::string>("/IPG/Publishers/CarState/Topic",                this->topics.carstate,      "/carmaker/state");
    nh->param<std::string>("/IPG/Publishers/Lidar/Topic",                   this->topics.lidar,         "/carmaker/pointcloud");
    nh->param<std::string>("/IPG/Publishers/Sensors/Odom/Topic",            this->topics.odom,          "/carmaker/Sensors/vectornav/Odom");
    nh->param<std::string>("/IPG/Publishers/Sensors/Pose/Topic",            this->topics.pose,          "/carmaker/Sensors/vectornav/Pose");
    nh->param<std::string>("/IPG/Publishers/Sensors/IMU/Topic",             this->topics.imu,           "/carmaker/Sensors/vectornav/IMU");
    nh->param<std::string>("/IPG/Publishers/Sensors/SBG/Front/Topic",       this->topics.sbg_front,     "/carmaker/Sensors/SBG/Front/IMU");
    nh->param<std::string>("/IPG/Publishers/Sensors/SBG/Rear/Topic",        this->topics.sbg_rear,      "/carmaker/Sensors/SBG/Rear/IMU");
    nh->param<std::string>("/IPG/Publishers/Sensors/Steering/Topic",        this->topics.steer_sensor,  "/carmaker/Sensors/Steering");
    nh->param<std::string>("/IPG/Publishers/Estimation/Kinematic/Topic",    this->topics.kinematic,     "/carmaker/Estimation/Kinematic");
    nh->param<std::string>("/IPG/Publishers/Estimation/Dynamic/Topic",      this->topics.dynamic,       "/carmaker/Estimation/Dynamic");
    nh->param<std::string>("/IPG/Publishers/Inverters/FL/Topic",            this->topics.inverterFL,    "/carmaker/Inverter/FrontLeftMotor");
    nh->param<std::string>("/IPG/Publishers/Inverters/FR/Topic",            this->topics.inverterFR,    "/carmaker/Inverter/FrontRightMotor");
    nh->param<std::string>("/IPG/Publishers/Inverters/RL/Topic",            this->topics.inverterRL,    "/carmaker/Inverter/RearLeftMotor");
    nh->param<std::string>("/IPG/Publishers/Inverters/RR/Topic",            this->topics.inverterRR,    "/carmaker/Inverter/RearRightMotor");
    nh->param<std::string>("/IPG/Publishers/Battery/Topic",                 this->topics.battery,       "/carmaker/Sensors/EM_ISBH");
    nh->param<std::string>("/IPG/Publishers/Pressure/Topic",                this->topics.pressure,      "/carmaker/Sensors/Pressure");
    nh->param<std::string>("/IPG/Publishers/Pedals/Throttle/Topic",         this->topics.throttle,      "/carmaker/Sensors/ThrottleEncoder");
    nh->param<std::string>("/IPG/Publishers/Pedals/Brake/Topic",            this->topics.brake,         "/carmaker/Sensors/BrakeEncoder");

    // Config
    nh->param<bool>("/IPG/Config/LateralCtrl",      this->config.lateralCtrl,       false);
    nh->param<bool>("/IPG/Config/Driver",           this->config.driver,            false);
    nh->param<bool>("/IPG/Config/HidraulicBrake",   this->config.hidraulic_brake,   true);
    nh->param<bool>("/IPG/Config/EBS",              this->config.EBS,               true);
    nh->param<int>("/IPG/Config/LLC",               this->config.LLC,               0);
    nh->param<bool>("/IPG/Config/4WD",              this->config.wd4,               true);

    // Noise
    nh->param<double>("/IPG/Noise/Odom/Mean", this->noise.mean_odom, 0.0);
    nh->param<double>("/IPG/Noise/Odom/Std",  this->noise.std_odom,  0.0);
    nh->param<double>("/IPG/Noise/IMU/Mean",  this->noise.mean_imu,  0.0);
    nh->param<double>("/IPG/Noise/IMU/Std",   this->noise.std_imu,   0.0);

    // Steering
    nh->param<double>("/IPG/Steering/MaxAtWheel", this->steering.maxAtWheel, 23.0);
    nh->param<double>("/IPG/Steering/MinAtWheel", this->steering.minAtWheel, -23.0);
        this->steering.maxAtWheel *= M_PI/180.0; // change from [º] to [rad]
        this->steering.minAtWheel *= M_PI/180.0;

}