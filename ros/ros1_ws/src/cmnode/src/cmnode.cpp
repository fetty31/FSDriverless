/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

/*!
 ******************************************************************************
                Main CarMaker ROS Node for BCN eMotorsport  
 ******************************************************************************
 */

#include "cmnode.h"

using namespace cm_yaml;

using cm_ros::CMNode;
using CMJob::Log;

CMNode::CMNode() {}
CMNode::~CMNode() { delete noise;}

/* Car Commands Subscriber Callback */
void CMNode::CarCommandsCallback(const ipg_msgs::CarCommands::ConstPtr &msg) {

    // Currently not used
    // VC.selector_ctrl   = msg->selector_ctrl;
    // VC.steer_ang_vel   = msg->steer_ang_vel;
    // VC.steer_ang_acc   = msg->steer_ang_acc;

    if(msg->motor >= 0){
      VC.gas     = std::min(1.0, (double) msg->motor);
      VC.brake   = 0.0;
    }else{
      VC.gas     = 0.0;
      VC.brake   = std::min(1.0, (double) fabs(msg->motor));
    }

    if(!paramsPtr->config.lateralCtrl) VC.steer_ang = this->correctSteering(msg->steering);
    VC.use_vc = true; // Tailored MPC is driving the car :) (IPG driver will be deactivated)

}

/* Steering Commands Subscriber Callback (used for decoupled control pipeline)*/
void CMNode::SteeringCallback(const ipg_msgs::CarCommands::ConstPtr &msg) { 
  
  if(paramsPtr->config.lateralCtrl) VC.steer_ang = this->correctSteering(msg->steering); 

}

void CMNode::FinishCallback(const std_msgs::Bool::ConstPtr &msg){
  if(paramsPtr->config.EBS) this->finishFlag = msg->data;
}

/* Motor Commands Subscriber Callback */
void CMNode::MotorCallback(const ipg_msgs::MotorInput::ConstPtr &msg, unsigned int pos) {

  if(VC.use_4wd){
    PTC.target_trq[pos] = m2w_trq(msg->targetTorque); // Transform motor to wheel values
    PTC.target_vel[pos] = m2w_vel(msg->setpointVelocity);
  }else{
    PTC.target_trq[pos] = (pos < 2) ? 0.0 : m2w_trq(msg->targetTorque); 
    PTC.target_vel[pos] = (pos < 2) ? 0.0 : m2w_vel(msg->setpointVelocity);
  }

  PTC.receiving = true;
}

/* Car State Publisher Callback */
void CMNode::CarStateFillMsg(ipg_msgs::CarState::Ptr& msg) {

      // time stamp
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = "global";

    // steering
  msg->steering = (double) (Car.Susp[0].SteerAngle + Car.Susp[1].SteerAngle)/2.0;

    // heading
  msg->odom.heading = this->correctHeading(Vehicle.Yaw);
  /* NOTE: IPG starts the simulation with 180º of heading (yaw) and [-inf, inf]. 
      We correct this heading so it starts at 0º and [-pi, pi] */ 

    // acceleration
  msg->odom.acceleration.x = Car.GenBdy1.a_1[0];
  msg->odom.acceleration.y = Car.GenBdy1.a_1[1];
  msg->odom.acceleration.z = Car.GenBdy1.a_1[2];

    // position
  msg->odom.position.x = Vehicle.PoI_Pos[0];
  msg->odom.position.y = Vehicle.PoI_Pos[1];
  msg->odom.position.z = Vehicle.PoI_Pos[2];

    // velocity
  msg->odom.velocity.x = Vehicle.PoI_Vel_1[0];
  msg->odom.velocity.y = Vehicle.PoI_Vel_1[1];
  msg->odom.velocity.w = Vehicle.YawRate;

  if(Ninertial >= 1){ /* If we have an Inertial Sensor available, we publish its data */

      // acceleration from sensor
    msg->odom.acceleration.x = InertialSensor[0].Acc_B[0];
    msg->odom.acceleration.y = InertialSensor[0].Acc_B[1];
    msg->odom.acceleration.z = InertialSensor[0].Acc_B[2];

      // position from sensor
    msg->odom.position.x = InertialSensor[0].Pos_0[0];
    msg->odom.position.y = InertialSensor[0].Pos_0[1];
    msg->odom.position.z = InertialSensor[0].Pos_0[2];

      // velocity from sensor
    msg->odom.velocity.x = InertialSensor[0].Vel_B[0];
    msg->odom.velocity.y = InertialSensor[0].Vel_B[1];
    msg->odom.velocity.w = InertialSensor[0].Omega_B[2];
  }

  // Add noise to Odom
  this->noise->add_noise( &(msg->odom.velocity.x) );
  this->noise->add_noise( &(msg->odom.velocity.y) );
  this->noise->add_noise( &(msg->odom.velocity.w) );

  // Add noise to IMU
  this->noise->add_noise( &(msg->odom.acceleration.x), this->paramsPtr->noise.mean_imu, this->paramsPtr->noise.std_imu);
  this->noise->add_noise( &(msg->odom.acceleration.y), this->paramsPtr->noise.mean_imu, this->paramsPtr->noise.std_imu);
  this->noise->add_noise( &(msg->odom.acceleration.z), this->paramsPtr->noise.mean_imu, this->paramsPtr->noise.std_imu);

  this->correctPosition( &(msg->odom.position.x), &(msg->odom.position.y), &(msg->odom.position.z) );
  /* NOTE: correctPosition sets initial position at 0,0,0 and rotates its frame so coordinates follow the 0º initial heading stated before*/

  // TF base_link
  double roll = this->correctPose(Vehicle.Roll);
  double pitch = this->correctPose(Vehicle.Pitch);
  tf::Quaternion q;
  q.setRPY(roll, pitch, msg->odom.heading);
  this->pubTF("global", "base_link", &(msg->odom.position.x), &(msg->odom.position.y), &(msg->odom.position.z), q);

  // this->pubTF("global", "base_link", &(msg->odom.position.x), &(msg->odom.position.y), &(msg->odom.position.z), &(msg->odom.heading) );

}

/* Car Odometry Publisher Callback */
void CMNode::OdomFillMsg(nav_msgs::Odometry::Ptr& msg) {

  // time stamp
  msg->header.stamp = ros::Time::now();

  if(Ninertial >= 1){

    // Twist
    msg->twist.twist.linear.x = InertialSensor[0].Vel_B[0];
    msg->twist.twist.linear.y = InertialSensor[0].Vel_B[1];
    msg->twist.twist.linear.z = InertialSensor[0].Vel_B[2];

    msg->twist.twist.angular.x = InertialSensor[0].Omega_B[0];
    msg->twist.twist.angular.y = InertialSensor[0].Omega_B[1];
    msg->twist.twist.angular.z = InertialSensor[0].Omega_B[2];

    // Pose
    msg->pose.pose.position.x = InertialSensor[0].Pos_0[0]; // no corrected position
    msg->pose.pose.position.y = InertialSensor[0].Pos_0[1];
    msg->pose.pose.position.z = InertialSensor[0].Pos_0[2];

    tf::Quaternion q;
    q.setRPY(Vehicle.Roll, Vehicle.Pitch, Vehicle.Yaw); // no corrected heading
    msg->pose.pose.orientation.x = q[0];
    msg->pose.pose.orientation.y = q[1];
    msg->pose.pose.orientation.z = q[2];
    msg->pose.pose.orientation.w = q[3];

  }
}

/* Car Odometry Publisher Callback */
void CMNode::OdomFillMsg(ipg_msgs::CarState::Ptr& msg) {

  // time stamp
  msg->header.stamp = ros::Time::now();

  if(Ninertial >= 1){

    // acceleration
    msg->odom.acceleration.x = InertialSensor[0].Acc_B[0];
    msg->odom.acceleration.y = InertialSensor[0].Acc_B[1];
    msg->odom.acceleration.z = InertialSensor[0].Acc_B[2];

    // velocity
    msg->odom.velocity.x = InertialSensor[0].Vel_B[0];
    msg->odom.velocity.y = InertialSensor[0].Vel_B[1];
    msg->odom.velocity.w = InertialSensor[0].Omega_B[2];

    // position
    msg->odom.position.x = InertialSensor[0].Pos_0[0]; // no corrected position
    msg->odom.position.y = InertialSensor[0].Pos_0[1];
    msg->odom.position.z = InertialSensor[0].Pos_0[2];

    // heading
    msg->odom.heading = Vehicle.Yaw; // no corrected heading
  }
}

/* Car IMU Publisher Callback */
void CMNode::IMUFillMsg(sensor_msgs::Imu::Ptr& msg, unsigned int sensor) {

  // time stamp
  msg->header.stamp = ros::Time::now();

  if(Ninertial >= 1){
    msg->linear_acceleration.x = InertialSensor[sensor].Acc_B[0];
    msg->linear_acceleration.y = InertialSensor[sensor].Acc_B[1];
    msg->linear_acceleration.z = InertialSensor[sensor].Acc_B[2];

    msg->angular_velocity.x = InertialSensor[sensor].Omega_B[0];
    msg->angular_velocity.y = InertialSensor[sensor].Omega_B[1];
    msg->angular_velocity.z = InertialSensor[sensor].Omega_B[2];
  }
}

/* Car Battery info Publisher Callback */
void CMNode::BatteryFillMsg(ipg_msgs::BatteryStatus::Ptr& msg){

  // time stamp
  msg->header.stamp = ros::Time::now();

  msg->Current.SensorValue      = PowerTrain.PowerSupplyIF.BattHV.Current;
  msg->Voltage.SensorValue      = PowerTrain.PowerSupplyIF.Voltage_HV1;
  msg->Power.SensorValue        = PowerTrain.PowerSupplyIF.Pwr_HV1;
  msg->Temperature.SensorValue  = PowerTrain.PowerSupplyIF.BattHV.Temp;

}

/* Throttle Position Publisher Callback */
void CMNode::ThrottleFillMsg(ipg_msgs::GeneralSensor::Ptr& msg) {

  msg->header.stamp = ros::Time::now(); // time stamp
  msg->SensorValue = DrivMan.Gas;
  msg->State = ipg_msgs::GeneralSensor::VALID;

}

/* Brake Position Publisher Callback */
void CMNode::BrakeFillMsg(ipg_msgs::GeneralSensor::Ptr& msg) {

  msg->header.stamp = ros::Time::now(); // time stamp
  msg->SensorValue = DrivMan.Brake;
  msg->State = ipg_msgs::GeneralSensor::VALID;

}

/* Brake pressures Publisher Callback */
void CMNode::PressureFillMsg(ipg_msgs::GeneralSensor::Ptr& msg) {

  msg->header.stamp = ros::Time::now();
  msg->SensorValue = Brake.HydBrakeIF.pMC; // total pressure in Master Cilinder [bar]
  msg->State = ipg_msgs::GeneralSensor::VALID;
}

/* Steering Position Publisher Callback */
void CMNode::SteerFillMsg(ipg_msgs::GeneralSensor::Ptr& msg) {

  msg->header.stamp = ros::Time::now(); // time stamp
  msg->SensorValue = (double) (Car.Susp[0].SteerAngle + Car.Susp[1].SteerAngle)/2.0;
  msg->State = ipg_msgs::GeneralSensor::VALID;
}

/* Kinematic Estimation Publisher Callback */
void CMNode::KinematicFillMsg(ipg_msgs::KinEstimate::Ptr& msg) {

  // time stamp
  msg->header.stamp = ros::Time::now();

  // Ground Truth Acceleration
  msg->acceleration.x = Car.GenBdy1.a_1[0];
  msg->acceleration.y = Car.GenBdy1.a_1[1];
  msg->acceleration.z = Car.GenBdy1.a_1[2];

  // Ground Truth Yaw Rate
  msg->velocity.w = Vehicle.YawRate;

  // Ground Truth Longitudinal Velocity
  msg->velocity.x   = Vehicle.PoI_Vel_1[0];
  msg->frontLeftVX  = Car.Tire[0].P_vel_W[0];
  msg->frontRightVX = Car.Tire[1].P_vel_W[0];
  msg->rearLeftVX   = Car.Tire[2].P_vel_W[0];
  msg->rearRightVX  = Car.Tire[3].P_vel_W[0];

  // Ground Truth Lateral Velocity
  msg->velocity.y   = Vehicle.PoI_Vel_1[1];
  msg->frontLeftVY  = Car.Tire[0].P_vel_W[1];
  msg->frontRightVY = Car.Tire[1].P_vel_W[1];
  msg->rearLeftVY   = Car.Tire[2].P_vel_W[1];
  msg->rearRightVY  = Car.Tire[3].P_vel_W[1];

  // Ground Truth Slip Angles
  msg->frontLeftSA  = Car.Tire[0].SlipAngle; 
  msg->frontRightSA = Car.Tire[1].SlipAngle;
  msg->rearLeftSA   = Car.Tire[2].SlipAngle;
  msg->rearRightSA  = Car.Tire[3].SlipAngle;

  // Ground Truth Slip Ratios
  msg->frontLeftSR  = Car.Tire[0].LongSlip;
  msg->frontRightSR = Car.Tire[1].LongSlip;
  msg->rearLeftSR   = Car.Tire[2].LongSlip;
  msg->rearRightSR  = Car.Tire[3].LongSlip;

}

/* Dynamic Estimation Publisher Callback */
void CMNode::DynamicFillMsg(ipg_msgs::DynEstimate::Ptr& msg) {

  // time stamp
  msg->header.stamp = ros::Time::now();

  // Longitudinal Forces
  msg->frontLeftFX  = Car.Tire[0].Frc_W[0];
  msg->frontRightFX = Car.Tire[1].Frc_W[0];
  msg->rearLeftFX   = Car.Tire[2].Frc_W[0];
  msg->rearRightFX  = Car.Tire[3].Frc_W[0];

  // Lateral Forces
  msg->frontLeftFY  = Car.Tire[0].Frc_W[1];
  msg->frontRightFY = Car.Tire[1].Frc_W[1];
  msg->rearLeftFY   = Car.Tire[2].Frc_W[1];
  msg->rearRightFY  = Car.Tire[3].Frc_W[1];

  // Vertial Forces
  msg->frontLeftFZ  = Car.Tire[0].Frc_W[2];
  msg->frontRightFZ = Car.Tire[1].Frc_W[2];
  msg->rearLeftFZ   = Car.Tire[2].Frc_W[2];
  msg->rearRightFZ  = Car.Tire[3].Frc_W[2];

}

/* Inverter (Motor) Data Publisher Callback */
void CMNode::InverterFillMsg(ipg_msgs::MotorOutput::Ptr& msg, unsigned int pos) {

  // time stamp
  msg->header.stamp = ros::Time::now();

  if(VC.use_4wd){
    msg->actualTorque   = PowerTrain.MotorIF[pos].Trq/PTC.ratio;
    msg->actualVelocity = PowerTrain.MotorIF[pos].rotv*PTC.ratio;
    msg->torqueEnable = true;
  }else{
    msg->actualTorque   = (pos < 2) ? 0.0 : PowerTrain.MotorIF[pos].Trq/PTC.ratio;
    msg->actualVelocity = (pos < 2) ? 0.0 : PowerTrain.MotorIF[pos].rotv*PTC.ratio;
    msg->torqueEnable = (pos < 2) ? false : true;
  }

  msg->motorTemperature    = 40.0; // hardcoded [ºC]
  msg->inverterTemperature = 40.0; // hardcoded [ºC]

}

/* Lidar Publisher Callback */
void CMNode::pointcloudFillMsg(sensor_msgs::PointCloud::Ptr& msg) {

  if (Sensor.LidarRSI.Active) {   

    geometry_msgs::Point32 points;
    sensor_msgs::ChannelFloat32 channels;
    channels.name = "intensity";

    /* Clear vector data to avoid overflows */
    msg->points.clear();
    msg->channels.clear();

    /* Populate Intensity channel data points */
    for (int i = 0; i < LidarRSI[0].nScanPoints; i++) {

      const int beam_id = LidarRSI[0].ScanPoint[i].BeamID;
      const double azimuth = angles::from_degrees(Sensor.LidarRSI.BeamTable[4*Sensor.LidarRSI.rows + beam_id]);
      const double elevation = angles::from_degrees(Sensor.LidarRSI.BeamTable[5*Sensor.LidarRSI.rows + beam_id]);
      const double ray_length = 0.5 * LidarRSI[0].ScanPoint[i].LengthOF; // length of flight is back and forth

      /* XYZ-coordinates of scan point */
      points.x = ray_length * cos(elevation) * cos(azimuth);
      points.y = ray_length * cos(elevation) * sin(azimuth);
      points.z = ray_length * sin(elevation);

      msg->points.push_back(points);
      channels.values.push_back(LidarRSI[0].ScanPoint[i].Intensity);
    }
    msg->channels.push_back(channels);
    msg->header.frame_id = Sensor.LidarRSI.frame_id;
    msg->header.stamp = ros::Time(LidarRSI[0].ScanTime);

  }
}

int CMNode::userInit() {

  synth_delay_ = 1e-6;

  printSmth("  -> Initializing service client /tracker/reset");
  srv_accum_ = nhp_->serviceClient<tracker::Init>("/tracker/reset");

  return 1;
}

int CMNode::userVehicleControlCalc(const double& dt)
{
    bool hidraulic = this->paramsPtr->config.hidraulic_brake;
    bool ebs = (bool) (this->finishFlag && this->paramsPtr->config.EBS); // only use EBS in finish state

    if(!VC.use_vc){ // If we want to use the driver, assign VC to DM quants 

      VehicleControl.SelectorCtrl     = DrivMan.SelectorCtrl;

      VehicleControl.Gas              = DrivMan.Gas;
      VehicleControl.Brake            = DrivMan.Brake;

      VehicleControl.Steering.Ang     = DrivMan.Steering.Ang;
      VehicleControl.Steering.AngVel  = DrivMan.Steering.AngVel;
      VehicleControl.Steering.AngAcc  = DrivMan.Steering.AngAcc;

    }else{ 

        this->printOnce("Using AS driving!");

        /* Assign ROS Topics data to CM Vehicle Control Quantities */
        /* -->Automatic transmission PRNDL selection */
        VehicleControl.SelectorCtrl     = VC.selector_ctrl; // left null
        VehicleControl.Gas              = 0.0; // overwrite VehicleControl commands
        VehicleControl.Brake            = 0.0;

        if(!VC.use_llc){ // If we are not using any self designed Low Level Control pipeline (let IPG's LLC handle it)

          /* -->Pedal Positions (add clutch if needed for manual) */
          VehicleControl.Gas              = VC.gas;
          VehicleControl.Brake            = VC.brake;

        }else if(hidraulic){
          VehicleControl.Brake = VC.brake;
          printSmth("Hidraulic switched on!");
        } // Otherwise LLC will be subscibed to Car Commands, so no need to fill VehicleControl's Gas & Brake

        if(ebs){
          VehicleControl.Brake = 1.0;
          printSmth("EBS switched on!");
        }

        /* -->Steer by angle inputs (add steer torque if using GenTorque model) */
        VehicleControl.Steering.Ang     = VC.steer_ang;
        VehicleControl.Steering.AngVel  = VC.steer_ang_vel; // left null
        VehicleControl.Steering.AngAcc  = VC.steer_ang_acc; // left null

    }
    return 1;
}

int CMNode::userCalc(const double& dt) {

  if(PTC.receiving){ // Assign Motor Torque commands

    if(this->paramsPtr->config.LLC == 1 || this->paramsPtr->config.LLC == 3){
      PowerTrain.TransmCU_IF.GearBoxM_Out[0].Trq_DriveSrc_trg = PTC.target_trq[0];
      PowerTrain.TransmCU_IF.GearBoxM_Out[1].Trq_DriveSrc_trg = PTC.target_trq[1];
      PowerTrain.TransmCU_IF.GearBoxM_Out[2].Trq_DriveSrc_trg = PTC.target_trq[2];
      PowerTrain.TransmCU_IF.GearBoxM_Out[3].Trq_DriveSrc_trg = PTC.target_trq[3];
    }
    if(this->paramsPtr->config.LLC >= 2){
      PowerTrain.ControlIF.MotorOut[0].rotv_trg = PTC.target_vel[0];
      PowerTrain.ControlIF.MotorOut[1].rotv_trg = PTC.target_vel[1];
      PowerTrain.ControlIF.MotorOut[2].rotv_trg = PTC.target_vel[2];
      PowerTrain.ControlIF.MotorOut[3].rotv_trg = PTC.target_vel[3];
    }
  }

  PTC.receiving = false;

  return 1;
}

void CMNode::userDeclQuants() {

  printSmth("userDeclQuants");

  tDDefault* df = DDefaultCreate("CMRosIF.HelloCM.");
  DDefULong(df, "CycleNoRel", "ms", &rel_cycle_num_, DVA_None);
  DDefDouble4(df, "SynthDelay", "s", &synth_delay_, DVA_IO_In);
  DDefaultDelete(df);
}

int CMNode::userTestrunStartAtBegin() {

  tErrorMsg *errv = nullptr;
  char sbuf[512];
  char *str         = nullptr;
  int idxC, idxS, idxP, ref;

  double def3c[]    = {0, 0, 0};             // Default 3-col table data

  // Call external nodes reset service
  if (!srv_accum_.exists()) {
    Log::printError(EC_Sim,
                    "ROS service is not ready! Please start external ROS node "
                    "providing service '" + srv_accum_.getService() + "'!");
    node_mode_ = NodeMode::kDisabled;
    return -1;
  }

  printSmth("  -> Sending service requests");

  tracker::Init srv_init;
  if (!srv_accum_.call(srv_init)) {
    Log::printError(EC_Sim, "Failed to call ROS service '" + srv_accum_.getService() + "'!");
    node_mode_ = NodeMode::kDisabled;
    return -1;
  }

  // Create params new instance (update parameters)
  std::shared_ptr<Params> paramsPtr0(new Params(nhp_));
  this->paramsPtr.reset();
  this->paramsPtr = std::move(paramsPtr0); // in c++11 we cannot use std::make_unique() :(
  Log::printLog("  -> Params updated!");
  printSmth(paramsPtr->config.LLC,                "    -> Low Level Control: ");
  printSmth(paramsPtr->config.driver,             "    -> Driver: ");
  printSmth(paramsPtr->config.hidraulic_brake,    "    -> Hidraulic Brake: ");
  printSmth(paramsPtr->config.EBS,                "    -> EBS: ");
  printSmth(paramsPtr->config.lateralCtrl,        "    -> Lateral Control: ");
  printSmth(paramsPtr->config.wd4,                "    -> 4WD: ");

  // Set up noise generator
  delete noise;
  noise = new noise::Gaussian(paramsPtr->noise.mean_odom, paramsPtr->noise.std_odom);

  // Do we want an IPG driver?
  if(!paramsPtr->config.driver) VC.use_vc = true; // we dont
  else VC.use_vc = false;

  // Are we using our self designed Low Level Control?
  if(paramsPtr->config.LLC == 0) VC.use_llc = false; // we will use IPG default controllers
  else VC.use_llc = true;

  // Are we using 4WD or 2WD?
  if(paramsPtr->config.wd4) VC.use_4wd = true;
  else VC.use_4wd = false;

  // Reset Vehicle Control (VC) variables
  VC.gas = 0.0;
  VC.brake = 0.0;

  // Reset Power Train Control (PTC) variables
  PTC.receiving = false;
  for(unsigned int i=0; i<4; i++){
    PTC.target_trq[i] = 0.0;
    PTC.target_vel[i] = 0.0;
  }

  // We must set the initial heading and position once again
  this->headingFlag = false;
  this->posFlag = false;

  this->finishFlag = false;

  // Check for inertial sensor once again
  this->Ninertial = 0;

// ---------------------------------- PUBLISHERS/ LIDAR -------------------------------------------

  /* Read sensor info from Vehicle InfoFile */
  tInfos *Inf_Vehicle = nullptr;
  Inf_Vehicle = InfoNew();

  const char *FName;
  FName = InfoGetFilename(SimCore.Vhcl.Inf);

  int VehicleInfo_Err = iRead2(&errv, Inf_Vehicle, FName, "SensorReadCode");
  printSmth<int>(VehicleInfo_Err, "VehicleInfo_Err ");

  if (VehicleInfo_Err == 0) {

    Sensor.nSensors = iGetIntOpt(Inf_Vehicle, "Sensor.N", 0);
    printSmth<int>(Sensor.nSensors, "Sensor.nSensors ");

    { /* set up LidarRSI publisher + job */

      /* Read Sensor Infofile for LidarRSI */
      tInfos *Inf_Sensor = nullptr;
      tErrorMsg *errs = nullptr;

      Inf_Sensor = InfoNew();
      int SensorInfo_Err = iRead2(&errs, Inf_Sensor, "Data/Sensor/LidarRSI_FS_autonomous", "LidarCode");
      printSmth<int>(SensorInfo_Err, "SensorInfo_Err ");

      /* Extract LidarRSI Parameters from Sensor Infofile */
      if (SensorInfo_Err == 0) {
        Sensor.LidarRSI.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        Sensor.LidarRSI.nBeams_h = Sensor.LidarRSI.nBeams[0];
        Sensor.LidarRSI.nBeams_v = Sensor.LidarRSI.nBeams[1];
        Sensor.LidarRSI.ret = iGetTableOpt(Inf_Sensor, "Beams", Sensor.LidarRSI.BeamTable, tableSize, 6, &Sensor.LidarRSI.rows);
        Sensor.LidarRSI.FOV_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1); 
        Sensor.LidarRSI.FOV_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1); 
        if (Sensor.LidarRSI.ret == 0) {
          Log("Beam table read successfully. Table consists of %i rows\n", Sensor.LidarRSI.rows);
          Log("FOVh = %f, FOVv = %f, nBeams_h = %f, nBeams_v = %f\n", 
            Sensor.LidarRSI.FOV_h[0] + Sensor.LidarRSI.FOV_h[1], 
            Sensor.LidarRSI.FOV_v[0] + Sensor.LidarRSI.FOV_v[1], 
            Sensor.LidarRSI.nBeams_h, 
            Sensor.LidarRSI.nBeams_v);
          Log("Beam Table --> First Azimuth Element = %f\n", Sensor.LidarRSI.BeamTable[4*Sensor.LidarRSI.rows]); 
          Log("Beam Table --> First Elevation Element = %f\n\n", Sensor.LidarRSI.BeamTable[5*Sensor.LidarRSI.rows]); 
        }
        Sensor.LidarRSI.ret = InfoDelete(Inf_Sensor);
      } 

      printSmth("Detected Sensors: ");
      
      idxP = -1;
      for (idxS = 0; idxS < Sensor.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        printSmth(str);
        if (!strcmp(str, "LidarRSI")) { /* Find the first LidarRSI in the Vehicle Infofile */
          idxP = ref; /* If the LidarRSI sensor is found, get its index */
        }
        if (!strcmp(str, "Inertial")){ /* Find the number of Inertial Sensors in the Vehicle Infofile */
          Ninertial++;
        }
      }

      /* Store LidarRSI sensor parameters */
      if (idxP != -1) {
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        Sensor.LidarRSI.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        Sensor.LidarRSI.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        Sensor.LidarRSI.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        Sensor.LidarRSI.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 10);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        Sensor.LidarRSI.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        /* set up Lidar publisher + job */
        typedef CMJob::RosPublisher<sensor_msgs::PointCloud::Ptr> Lidar;
        auto job = std::make_shared<Lidar>(nhp_, paramsPtr->topics.lidar);
        job->setCycleTime(Sensor.LidarRSI.UpdRate);
        job->setCycleOffset(Sensor.LidarRSI.nCycleOffset);
        job->registerCallback(&CMNode::pointcloudFillMsg, this);
        scheduler_.addJob(job);

        // Static TF lidar
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped staticMsg;
        tf2::Quaternion q;
        q.setRPY(angles::from_degrees(Sensor.LidarRSI.rot[0]), 
                  angles::from_degrees(Sensor.LidarRSI.rot[1]), 
                  angles::from_degrees(Sensor.LidarRSI.rot[2]));
        staticMsg.transform.rotation = tf2::toMsg(q);
        staticMsg.transform.translation = tf2::toMsg(tf2::Vector3( 
            Sensor.LidarRSI.pos[0],
            Sensor.LidarRSI.pos[1],
            Sensor.LidarRSI.pos[2] ));
        staticMsg.header.frame_id = "base_link";
        staticMsg.child_frame_id = "lidar";
        static_broadcaster.sendTransform(staticMsg);

      } else {
        Sensor.LidarRSI.Active                   = 0;

        // Static TF lidar (fake)
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped staticMsg;
        tf2::Quaternion q;
        q.setRPY(0.0, 
                 0.0, 
                 0.0);
        staticMsg.transform.rotation = tf2::toMsg(q);
        staticMsg.transform.translation = tf2::toMsg(tf2::Vector3( 
            1.68,
            0.0,
            0.2 ));
        staticMsg.header.frame_id = "base_link";
        staticMsg.child_frame_id = "lidar";
        static_broadcaster.sendTransform(staticMsg);
      }
    } 
  }

// ---------------------------------- PUBLISHERS/ CAR STATE -------------------------------------------

  {  // set up CarState publisher + job
    int cycle_time = 1.0/paramsPtr->hz.carstate * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::CarState::Ptr> carstate_t;
    auto job = std::make_shared<carstate_t>(nhp_, paramsPtr->topics.carstate);
    job->setCycleTime(cycle_time); // car state should be published at 40 Hz
    job->registerCallback(&CMNode::CarStateFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ ODOM -------------------------------------------

  {  // set up Odom publisher + job
    int cycle_time = 1.0/paramsPtr->hz.odom * 1000; // [ms]
    typedef CMJob::RosPublisher<nav_msgs::Odometry::Ptr> odom_t;
    auto job = std::make_shared<odom_t>(nhp_, paramsPtr->topics.odom);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::OdomFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ POSE -------------------------------------------

  {  // set up Pose publisher + job
    int cycle_time = 1.0/paramsPtr->hz.pose * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::CarState::Ptr> state_t;
    auto job = std::make_shared<state_t>(nhp_, paramsPtr->topics.pose);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::OdomFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ IMU -------------------------------------------

  {  // set up IMU publisher + job
    int cycle_time = 1.0/paramsPtr->hz.imu * 1000; // [ms]
    typedef CMJob::RosPublisher<sensor_msgs::Imu::Ptr> imu_t;
    auto job = std::make_shared<imu_t>(nhp_, paramsPtr->topics.imu);
    auto f = boost::bind(&CMNode::IMUFillMsg, this, _1, 0);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ SBGs -------------------------------------------

  {  // set up SBG front publisher + job
    int cycle_time = 1.0/paramsPtr->hz.sbg_front * 1000; // [ms]
    typedef CMJob::RosPublisher<sensor_msgs::Imu::Ptr> imu_t;
    auto job = std::make_shared<imu_t>(nhp_, paramsPtr->topics.sbg_front);
    auto f = boost::bind(&CMNode::IMUFillMsg, this, _1, 1);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    if(Ninertial >= 2) scheduler_.addJob(job);
  }

  {  // set up SBG rear publisher + job
    int cycle_time = 1.0/paramsPtr->hz.sbg_rear * 1000; // [ms]
    typedef CMJob::RosPublisher<sensor_msgs::Imu::Ptr> imu_t;
    auto job = std::make_shared<imu_t>(nhp_, paramsPtr->topics.sbg_rear);
    auto f = boost::bind(&CMNode::IMUFillMsg, this, _1, 2);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    if(Ninertial >= 3) scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ BATTERY -------------------------------------------

  {  // set up Battery publisher + job
    int cycle_time = 1.0/paramsPtr->hz.battery * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::BatteryStatus::Ptr> battery_t;
    auto job = std::make_shared<battery_t>(nhp_, paramsPtr->topics.battery);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::BatteryFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ PEDALS -------------------------------------------

  {  // set up Throttle publisher + job
    int cycle_time = 1.0/paramsPtr->hz.throttle * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::GeneralSensor::Ptr> throttle_t;
    auto job = std::make_shared<throttle_t>(nhp_, paramsPtr->topics.throttle);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::ThrottleFillMsg, this);
    scheduler_.addJob(job);
  }

  {  // set up Brake publisher + job
    int cycle_time = 1.0/paramsPtr->hz.brake * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::GeneralSensor::Ptr> brake_t;
    auto job = std::make_shared<brake_t>(nhp_, paramsPtr->topics.brake);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::BrakeFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ PRESSURES -------------------------------------------

  {  // set up Pressure publisher + job
    int cycle_time = 1.0/paramsPtr->hz.pressure * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::GeneralSensor::Ptr> pressure_t;
    auto job = std::make_shared<pressure_t>(nhp_, paramsPtr->topics.pressure);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::PressureFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ STEERING DATA -------------------------------------------

  {  // set up Steering Data publisher + job
    int cycle_time = 1.0/paramsPtr->hz.steer_sensor * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::GeneralSensor::Ptr> sensor_t;
    auto job = std::make_shared<sensor_t>(nhp_, paramsPtr->topics.steer_sensor);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::SteerFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ KINEMATIC ESTIMATION -------------------------------------------

  {  // set up Kinematic Estimation publisher + job
    int cycle_time = 1.0/paramsPtr->hz.kinematic * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::KinEstimate::Ptr> estimation_t;
    auto job = std::make_shared<estimation_t>(nhp_, paramsPtr->topics.kinematic);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::KinematicFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ DYNAMIC ESTIMATION -------------------------------------------

  {  // set up Dynamic Estimation publisher + job
    int cycle_time = 1.0/paramsPtr->hz.dynamic * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::DynEstimate::Ptr> estimation_t;
    auto job = std::make_shared<estimation_t>(nhp_, paramsPtr->topics.dynamic);
    job->setCycleTime(cycle_time);
    job->registerCallback(&CMNode::DynamicFillMsg, this);
    scheduler_.addJob(job);
  }

// ---------------------------------- PUBLISHERS/ INVERTER DATA -------------------------------------------

  {  // set up Inverter FL Data publisher + job
    int cycle_time = 1.0/paramsPtr->hz.inverterFL * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::MotorOutput::Ptr> inverter_t;
    auto job = std::make_shared<inverter_t>(nhp_, paramsPtr->topics.inverterFL);
    auto f = boost::bind(&CMNode::InverterFillMsg, this, _1, 0);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    scheduler_.addJob(job);
  }

  {  // set up Inverter FR Data publisher + job
    int cycle_time = 1.0/paramsPtr->hz.inverterFR * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::MotorOutput::Ptr> inverter_t;
    auto job = std::make_shared<inverter_t>(nhp_, paramsPtr->topics.inverterFR);
    auto f = boost::bind(&CMNode::InverterFillMsg, this, _1, 1);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    scheduler_.addJob(job);
  }

  {  // set up Inverter RL Data publisher + job
    int cycle_time = 1.0/paramsPtr->hz.inverterRL * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::MotorOutput::Ptr> inverter_t;
    auto job = std::make_shared<inverter_t>(nhp_, paramsPtr->topics.inverterRL);
    auto f = boost::bind(&CMNode::InverterFillMsg, this, _1, 2);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    scheduler_.addJob(job);
  }

  {  // set up Inverter RR Data publisher + job
    int cycle_time = 1.0/paramsPtr->hz.inverterRR * 1000; // [ms]
    typedef CMJob::RosPublisher<ipg_msgs::MotorOutput::Ptr> inverter_t;
    auto job = std::make_shared<inverter_t>(nhp_, paramsPtr->topics.inverterRR);
    auto f = boost::bind(&CMNode::InverterFillMsg, this, _1, 3);
    job->setCycleTime(cycle_time);
    job->registerCallback(f);
    scheduler_.addJob(job);
  }

// ---------------------------------- SUBSCRIBERS/ CAR COMMANDS -------------------------------------------

  // Clock Check var
  int clockCheck = 1;

  {  // set up Car Commands subscriber + job
    std::string topic = paramsPtr->topics.commands;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.commands * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<ipg_msgs::CarCommands::ConstPtr> carcommands_t;
    auto job = std::make_shared<carcommands_t>(job_type, synchronize, nhp_, topic);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(&CMNode::CarCommandsCallback, this);
    scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

// ---------------------------------- SUBSCRIBERS/ STEERING COMMANDS -------------------------------------------

  {  // set up Steering Commands subscriber + job
    std::string topic = paramsPtr->topics.steering;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.steering * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<ipg_msgs::CarCommands::ConstPtr> carcommands_t;
    auto job = std::make_shared<carcommands_t>(job_type, synchronize, nhp_, topic);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(&CMNode::SteeringCallback, this);
    scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

// ---------------------------------- SUBSCRIBERS/ FINISH FLAG -------------------------------------------

  {  // set up Steering Commands subscriber + job
    std::string topic = paramsPtr->topics.finish;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.finish * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<std_msgs::Bool::ConstPtr> finish_t;
    auto job = std::make_shared<finish_t>(job_type, synchronize, nhp_, topic);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(&CMNode::FinishCallback, this);
    scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

// ---------------------------------- SUBSCRIBERS/ MOTOR COMMANDS -------------------------------------------

  {  // set up Motor FL Commands subscriber + job
    std::string topic = paramsPtr->topics.motorFL;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.motorFL * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<ipg_msgs::MotorInput::ConstPtr> motor_t;
    auto job = std::make_shared<motor_t>(job_type, synchronize, nhp_, topic);
    auto f = boost::bind(&CMNode::MotorCallback, this, _1, 0);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(f);
    if(paramsPtr->config.LLC > 0) scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

  {  // set up Motor FR Commands subscriber + job
    std::string topic = paramsPtr->topics.motorFR;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.motorFR * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<ipg_msgs::MotorInput::ConstPtr> motor_t;
    auto job = std::make_shared<motor_t>(job_type, synchronize, nhp_, topic);
    auto f = boost::bind(&CMNode::MotorCallback, this, _1, 1);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(f);
    if(paramsPtr->config.LLC > 0) scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

  {  // set up Motor FR Commands subscriber + job
    std::string topic = paramsPtr->topics.motorRL;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.motorRL * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<ipg_msgs::MotorInput::ConstPtr> motor_t;
    auto job = std::make_shared<motor_t>(job_type, synchronize, nhp_, topic);
    auto f = boost::bind(&CMNode::MotorCallback, this, _1, 2);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(f);
    if(paramsPtr->config.LLC > 0) scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

  {  // set up Motor FR Commands subscriber + job
    std::string topic = paramsPtr->topics.motorRR;
    bool synchronize = (sync_mode_ == SyncMode::kTopic);
    CMJob::JobType job_type =
        synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

    int cycle_time = 1.0/paramsPtr->hz.motorRR * 1000; // we want to check for new msgs as fast as possible [ms]

    typedef CMJob::RosSubscriber<ipg_msgs::MotorInput::ConstPtr> motor_t;
    auto job = std::make_shared<motor_t>(job_type, synchronize, nhp_, topic);
    auto f = boost::bind(&CMNode::MotorCallback, this, _1, 3);
    job->setCycleTime(static_cast<unsigned long>(cycle_time));
    job->skipFirstCycles(1);
    job->setTimeoutTime(max_sync_time_);
    job->registerCallback(f);
    if(paramsPtr->config.LLC > 0) scheduler_.addJob(job);

    clockCheck = this->clockTimeCheck(cycle_time, topic.c_str());
  }

  return clockCheck;
}

int CMNode::userTestrunEnd() {

  // scheduler_.deleteJobs(); // not working, no idea why
  
  std::shared_ptr<Params> paramsPtr0(new Params(nhp_)); // work around because we can't acces to paramsPtr object
  scheduler_.deleteJob(paramsPtr0->topics.carstate);
  scheduler_.deleteJob(paramsPtr0->topics.commands);
  scheduler_.deleteJob(paramsPtr0->topics.steering);
  scheduler_.deleteJob(paramsPtr0->topics.finish);
  scheduler_.deleteJob(paramsPtr0->topics.odom);
  scheduler_.deleteJob(paramsPtr0->topics.imu);
  scheduler_.deleteJob(paramsPtr0->topics.kinematic);
  scheduler_.deleteJob(paramsPtr0->topics.dynamic);
  scheduler_.deleteJob(paramsPtr0->topics.battery);
  scheduler_.deleteJob(paramsPtr0->topics.inverterFL);
  scheduler_.deleteJob(paramsPtr0->topics.inverterFR);
  scheduler_.deleteJob(paramsPtr0->topics.inverterRL);
  scheduler_.deleteJob(paramsPtr0->topics.inverterRR);
  scheduler_.deleteJob(paramsPtr0->topics.steer_sensor);
  scheduler_.deleteJob(paramsPtr0->topics.throttle);
  scheduler_.deleteJob(paramsPtr0->topics.brake);
  scheduler_.deleteJob(paramsPtr0->topics.pressure);
  if(Ninertial >= 2) scheduler_.deleteJob(paramsPtr0->topics.sbg_front);
  if(Ninertial >= 3) scheduler_.deleteJob(paramsPtr0->topics.sbg_rear);
  if(Sensor.LidarRSI.Active) scheduler_.deleteJob(paramsPtr0->topics.lidar);
  if(paramsPtr0->config.LLC > 0){
    scheduler_.deleteJob(paramsPtr0->topics.motorFL);
    scheduler_.deleteJob(paramsPtr0->topics.motorFR);
    scheduler_.deleteJob(paramsPtr0->topics.motorRL);
    scheduler_.deleteJob(paramsPtr0->topics.motorRR);
  }
  return 1;
}

int CMNode::customFunction(const int input, char* output) {
  strcpy(output, __PRETTY_FUNCTION__);
  return clock_cycle_time_ + input;
}

template<typename mytype>
void CMNode::printSmth(mytype num, std::string comment){
  try{
    std::ostringstream s;
    s << num;
    std::string str = comment + s.str();
    Log::printLog(str);
  }catch (...){
    std::cout << boost::current_exception_diagnostic_information() << std::endl;
    Log::printLog( boost::current_exception_diagnostic_information() );
  }
}

int CMNode::clockTimeCheck(int cycle_time, const char* name){
    if (cycle_time % clock_cycle_time_ != 0 ||
      (cycle_time < clock_cycle_time_ && clock_cycle_time_ > 0)){
      node_mode_ = NodeMode::kDisabled;
      LogErrF(EC_Sim,
              "Ext. ROS node %s has an invalid cycle time! Expected multiple of "
              "%iums but got %ims",
              name, clock_cycle_time_, cycle_time);

      return -1;
    }else return 1;
}

double CMNode::correctHeading(double heading){

  // printSmth<double>(heading, "Heading: ");

  // First iteration --> set initial heading
  if(!this->headingFlag){
    initial_heading = heading;
    this->headingFlag = true;
  } 
  
  heading -= initial_heading; // at first iter is always 0.0
  heading = fmod(heading, 2*M_PI);

  // Transform heading into [-pi, pi]
  if(heading < -M_PI) heading += 2*M_PI;
  if(heading > M_PI) heading -= 2*M_PI;

  return heading;
}

double CMNode::correctPose(double phi){
  phi = fmod(phi, 2*M_PI);
  if(phi < -M_PI) phi += 2*M_PI;
  if(phi > M_PI) phi -= 2*M_PI;
  return phi;
}

double CMNode::correctSteering(double steering){ // from wheel frame to steering wheel frame [-270º, 270º] (linear approximation)

  // printSmth<double>(steering, "Steering at wheel: ");

  if(steering >= 0) steering *= 3.0/2.0*M_PI/(paramsPtr->steering.maxAtWheel);
  else steering *= 3.0/2.0*M_PI/fabs(paramsPtr->steering.minAtWheel);

  // printSmth<double>(steering, "Final Steering: ");

  return steering;
}

double CMNode::m2w_trq(double trq){

  trq *= PTC.ratio;
  return trq;
}

double CMNode::m2w_vel(double vel){

  vel /= PTC.ratio;
  return vel;
}

void CMNode::correctPosition(double* x, double* y, double* z){

  // First iteration --> set initial position
  if(!this->posFlag){
    initial_pos[0] = *x;
    initial_pos[1] = *y;
    initial_pos[2] = *z;
    this->posFlag = true;
  }

  // Correct positions
  *x -= initial_pos[0]; 
  *y -= initial_pos[1]; 
  *z -= initial_pos[2]; 

  // Rotate positions (ipg frame to global)
  double angle = -initial_heading;
  this->rotatePosition(x, y, &angle);

}

void CMNode::rotatePosition(double* x, double* y, double* angle){
  *x = *x * cos(*angle) - *y * sin(*angle);
  *y = *x * sin(*angle) + *y * cos(*angle);
}

void CMNode::pubTF(const std::string& frame, const std::string& child_frame, double* x, double* y, double* z, double* yaw /*== heading*/){

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(*x, *y, *z) );
  tf::Quaternion q;
  q.setRPY(0, 0, *yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, child_frame));

}

void CMNode::pubTF(const std::string& frame, const std::string& child_frame, double* x, double* y, double* z, tf::Quaternion q){
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(*x, *y, *z) );
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, child_frame));

}

// ---------------------------------- Extern functions -------------------------------------------

int CMNode_customFunction(void* intf, const int input, char* output) {
  return reinterpret_cast<CMNode*>(intf)->customFunction(input, output);
}

// Important: Use this macro to register the derived class as an interface with
// the CarMaker C++ Interface Loader module
REGISTER_CARMAKER_CPP_IF(CMNode)
