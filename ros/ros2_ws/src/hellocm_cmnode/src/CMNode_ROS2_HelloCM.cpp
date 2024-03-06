/*!
 ******************************************************************************
 **  CarMaker - Version 11.0.1
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 *
 *
 * ToDo:
 * - C++!!!
 * - ROS naming/way/namespaces
 * - parameter: CarMaker read, ROS set by service?
 *   -> ROS parameter mechanism seems better solution!
 * - node/topic/... destruction to allow dynamic load/unload
 *   when TestRun starts instead of initialization at CarMaker startup
 * - New Param_Get() function to read parameters from Infofile
 * - ...
 *
 */
#include "CMNode_ROS2_HelloCM.hpp"

using cm_ros::CMNodeHelloCM;
using CMJob::Log;

CMNodeHelloCM::CMNodeHelloCM() {}

/* Lidar RSI --> Number of Beams */
static const unsigned int tableSize = 15000 * 6;

/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Define a struct for the CameraRSI data */
typedef struct tCameraRSI {
    char*           name;
    int             number;
    double*         pos;                /*!< Mounting position on vehicle frame */
    double*         rot;                /*!< Mounting rotation on vehicle frame */
    double          FoV;
    double*         Resolution;         /*!< Resolution width and height */
    int             UpdRate;
    int             nCycleOffset;
    int             Socket;
} tCameraRSI;

/* Global struct for this Node */
static struct {
    struct {
        double         Duration;      /*!< Time spent for synchronization task */
        int            nCycles;       /*!< Number of cycles in synchronization loop */
    } Sync; /*!< Synchronization related information */

    struct {
        std::shared_ptr<tf2_ros::TransformBroadcaster> br;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> st_br;

        geometry_msgs::msg::TransformStamped Lidar;
        geometry_msgs::msg::TransformStamped Camera;
        geometry_msgs::msg::TransformStamped ObjectList;
    } TF;

    struct {
        struct {
        } Sub; /*!< Topics to be subscribed */
        struct {
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
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

    struct {
        bool                use_vc;
        int                 selector_ctrl;
        double              gas;
        double              brake;
        double              steer_ang;
        double              steer_ang_vel;
        double              steer_ang_acc;
    } VC; /*!< Vehicle Controls */

    struct {
        int nSensors;
    } Misc;
} CMNode;


/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
void CMNodeHelloCM::ext2cmCallback(hellocm_msgs::msg::Ext2CM::ConstSharedPtr msg) {
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(3);
  ss << "\t[" << rclcpp::Clock().now().seconds();
  if (nhp_->get_clock()->ros_time_is_active()) {
    ss << ", " << nhp_->now().seconds();
  }
  ss << "]: " << nhp_->get_fully_qualified_name() << ": Sub Msg: ";
  ss << "Time " << rclcpp::Time(msg->time).seconds() << "s, ";
  ss << "Cycle " << msg->cycleno;

  Log::printLog(ss.str());
}

/* Vehicle Control Subscriber Callback */
void CMNodeHelloCM::vehicleControlCallback(vehiclecontrol_msgs::msg::VehicleControl::ConstSharedPtr msg) {

    CMNode.VC.use_vc          = msg->use_vc;
    CMNode.VC.selector_ctrl   = msg->selector_ctrl;
    CMNode.VC.gas             = msg->gas;
    CMNode.VC.brake           = msg->brake;
    CMNode.VC.steer_ang       = msg->steer_ang;
    CMNode.VC.steer_ang_vel   = msg->steer_ang_vel;
    CMNode.VC.steer_ang_acc   = msg->steer_ang_acc;

/*
    in->Msg.key             = msg->key;
    in->Msg.sst             = msg->sst;

    in->Msg.brakelight      = msg->brakelight;
    in->Msg.daytimelights   = msg->daytimelights;
    in->Msg.foglights_left  = msg->foglights_left;
    in->Msg.foglights_right = msg->foglights_right;
    in->Msg.foglights_rear  = msg->foglights_rear;
    in->Msg.highbeams       = msg->highbeams;
    in->Msg.indicator_left  = msg->indicator_left;
    in->Msg.indicator_right = msg->indicator_right;
    in->Msg.lowbeams        = msg->lowbeams;
    in->Msg.parklight_left  = msg->parklight_left;
    in->Msg.parklight_right = msg->parklight_right;
    in->Msg.reverselights   = msg->reverselights;
*/
}


/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Basic Initialization
 * - e.g. create ROS Node, subscriptions, ...
 * - Return values
 *   - "rv <  0" = Error at initialization, CarMaker executable will stop
 *   - "rv >= 0" = Everything OK, CarMaker executable will continue
 *
 * Arguments:
 * - Inf        = Handle to CarMaker Infofile with parameters for this interface
 *                - Please note that pointer may change, e.g. before TestRun begins
 */
int
CMNodeHelloCM::userInit()
{
  CMNode.TF.br = std::make_shared<tf2_ros::TransformBroadcaster>(nhp_);
  CMNode.TF.st_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nhp_);

  Log::printLog("  -> Initializing service client /hellocm/init");
  srv_init_ = nhp_->create_client<hellocm_msgs::srv::Init>("/hellocm/init");
  param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(nhp_, "/hellocm/hellocm");

  return 1;
}

/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after CMRosIF_CMNode_DrivManCalc
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_VehicleControl_Calc()"
 */
int CMNodeHelloCM::userVehicleControlCalc(const double& dt)
{
    (void)dt;
    
    /* Assign VC to DM quants in case not written to... */
    VehicleControl.SelectorCtrl     = DrivMan.SelectorCtrl;

    VehicleControl.Gas              = DrivMan.Gas;
    VehicleControl.Brake            = DrivMan.Brake;

    VehicleControl.Steering.Ang     = DrivMan.Steering.Ang;
    VehicleControl.Steering.AngVel  = DrivMan.Steering.AngVel;
    VehicleControl.Steering.AngAcc  = DrivMan.Steering.AngAcc;

    if (SimCore.Time < 0.001) {
        CMNode.VC.use_vc = 0; //Reset back to DM at beginning of simulation...
    }

    if (CMNode.VC.use_vc == 1) {
            
        /* Assign ROS Topic to CM Vehicle Control Quantities */
        /* -->Automatic transmission PRNDL selection */
        VehicleControl.SelectorCtrl     = CMNode.VC.selector_ctrl;

        /* -->Pedal Positions (add clutch if needed for manual) */
        VehicleControl.Gas              = CMNode.VC.gas;
        VehicleControl.Brake            = CMNode.VC.brake;

        /* -->Steer by angle inputs (add steer torque if using GenTorque model) */
        VehicleControl.Steering.Ang     = CMNode.VC.steer_ang;
        VehicleControl.Steering.AngVel  = CMNode.VC.steer_ang_vel;
        VehicleControl.Steering.AngAcc  = CMNode.VC.steer_ang_acc;

    }

    return 1;
}

#ifdef __cplusplus
}
#endif

void CMNodeHelloCM::pointcloudFillMsg(sensor_msgs::msg::PointCloud& msg) {

  if (CMNode.Sensor.LidarRSI.Active) {   

    geometry_msgs::msg::Point32 points;
    sensor_msgs::msg::ChannelFloat32 channels;
    channels.name = "intensity";

    /* Clear vector data to avoid overflows */
    msg.points.clear();
    msg.channels.clear();

    /* Populate Intensity channel data points */
    for (int i = 0; i < LidarRSI[0].nScanPoints; i++) {

      const int beam_id = LidarRSI[0].ScanPoint[i].BeamID;
      const double azimuth = angles::from_degrees(CMNode.Sensor.LidarRSI.BeamTable[4*CMNode.Sensor.LidarRSI.rows + beam_id]);
      const double elevation = angles::from_degrees(CMNode.Sensor.LidarRSI.BeamTable[5*CMNode.Sensor.LidarRSI.rows + beam_id]);
      const double ray_length = 0.5 * LidarRSI[0].ScanPoint[i].LengthOF; // length of flight is back and forth

      /* XYZ-coordinates of scan point */
      points.x = ray_length * cos(elevation) * cos(azimuth);
      points.y = ray_length * cos(elevation) * sin(azimuth);
      points.z = ray_length * sin(elevation);

      msg.points.push_back(points);
      channels.values.push_back(LidarRSI[0].ScanPoint[i].Intensity);
    }
    msg.channels.push_back(channels);
    msg.header.frame_id = CMNode.TF.Lidar.child_frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(1e9 * LidarRSI[0].ScanTime), RCL_ROS_TIME);
  }
}

void CMNodeHelloCM::objectlistFillMsg(visualization_msgs::msg::MarkerArray& msg) {

  if (CMNode.Sensor.Object.Active) {   

    visualization_msgs::msg::Marker markers;
    rclcpp::Time object_sensor_stamp = rclcpp::Time(static_cast<int64_t>(1e9 * ObjectSensor[0].TimeStamp), RCL_ROS_TIME);

    /* Clear vector data to avoid overflows */
    msg.markers.clear();

    /* Display objects with a duration of the topics cycle time, so rviz forgets them when 
       they are not being published anymore: (80 + milliseconds) to seconds */
    rclcpp::Duration object_viz_time = rclcpp::Duration(static_cast<int64_t>(1e6 * 80 + CMNode.Sensor.Camera.UpdRate), RCL_ROS_TIME);

    for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++) {

      int obj_id = ObjectSensor[0].ObsvObjects[j];
      tObjectSensorObj *pOSO = ObjectSensor_GetObjectByObjId(0, obj_id);
      tTrafficObj *pObj = Traffic_GetByObjId(obj_id);

      markers.header.frame_id = CMNode.TF.ObjectList.child_frame_id;
      markers.id = pOSO->ObjId;
      markers.type = visualization_msgs::msg::Marker::CUBE;
      markers.action = visualization_msgs::msg::Marker::ADD;
      markers.lifetime = object_viz_time;
      markers.header.stamp = object_sensor_stamp;

      markers.scale.x = pOSO->l;
      markers.scale.y = pOSO->w;
      markers.scale.z = pOSO->h;

      /* white */
      markers.color.a = 0.7f;
      markers.color.r = 1.0;
      markers.color.g = 1.0;
      markers.color.b = 1.0;

      if (pOSO->dtct || pOSO->InLane){
        if (!strcmp(pObj->Cfg.Info, "Right Yellow Cone")) {
          /* yellow */
          markers.color.r = 1.0;
          markers.color.g = 1.0;
          markers.color.b = 0.0;
        } else if (!strcmp(pObj->Cfg.Info, "Left Blue Cone")) {
          /* blue */
          markers.color.r = 0.0;
          markers.color.g = 0.3;
          markers.color.b = 1.0;
        } else {
          /* orange start cones */
          markers.color.r = 1.0;
          markers.color.g = 0.6;
          markers.color.b = 0.0;
        }
      }

      /* Object rotation as quaternion */
      tf2::Quaternion rotation;
      rotation.setRPY(pOSO->RefPnt.r_zyx[0], pOSO->RefPnt.r_zyx[1], pOSO->RefPnt.r_zyx[2]);

      /* Apply rotation to vector pointing to the object's center */
      tf2::Vector3 obj_center = tf2::quatRotate(rotation, tf2::Vector3(0.5 * pOSO->l, 0, 0));

      markers.pose.position.x = pOSO->RefPnt.ds[0] + obj_center.getX();
      markers.pose.position.y = pOSO->RefPnt.ds[1] + obj_center.getY();
      markers.pose.position.z = pOSO->RefPnt.ds[2] + obj_center.getZ();
      markers.pose.orientation = tf2::toMsg(rotation);

      msg.markers.push_back(markers);
    } // for (int j = 0; j < ObjectSensor[0].nObsvObjects; j++)
  } // if (CMNode.Sensor.Object.Active)
}

void CMNodeHelloCM::cameraFillMsg(camera_msgs::msg::CameraDetectionArray& msg) {

  if (CMNode.Sensor.Camera.Active) {

    camera_msgs::msg::CameraDetection detections; 

    /* Clear vector data to avoid overflows */
    msg.detections.clear();

    for (int k = 0; k <= CameraSensor[0].nObj; k++) {

      detections.objid            = int32_t(CameraSensor[0].Obj[k].ObjID);
      detections.nvispixels       = int64_t(CameraSensor[0].Obj[k].nVisPixels);
      detections.confidence       = CameraSensor[0].Obj[k].Confidence;

      detections.objecttype       = CameraSensor[0].Obj[k].Type;

      detections.mbr_bl_x         = CameraSensor[0].Obj[k].MBR[0][0];
      detections.mbr_bl_y         = CameraSensor[0].Obj[k].MBR[0][1];
      detections.mbr_bl_z         = CameraSensor[0].Obj[k].MBR[0][2];
      detections.mbr_tr_x         = CameraSensor[0].Obj[k].MBR[1][0];
      detections.mbr_tr_y         = CameraSensor[0].Obj[k].MBR[1][1];
      detections.mbr_tr_z         = CameraSensor[0].Obj[k].MBR[1][2];

      detections.facing           = int8_t(CameraSensor[0].Obj[k].Facing);
      detections.lightstate       = int8_t(CameraSensor[0].Obj[k].LightState);

      detections.signmain_val0    = CameraSensor[0].Obj[k].SignMainVal[0];
      detections.signmain_val1    = CameraSensor[0].Obj[k].SignMainVal[1];

      detections.signsuppl1_val0  = CameraSensor[0].Obj[k].SignSuppl1Val[0];
      detections.signsuppl1_val1  = CameraSensor[0].Obj[k].SignSuppl1Val[1];

      detections.signsuppl2_val0  = CameraSensor[0].Obj[k].SignSuppl2Val[0];
      detections.signsuppl2_val1  = CameraSensor[0].Obj[k].SignSuppl2Val[1];

      msg.detections.push_back(detections);
    } // for (int k = 0; k <= CameraSensor[0].nObj; k++)
  } // if (CMNode.Sensor.Camera.Active)
}

void CMNodeHelloCM::cm2extFillMsg(hellocm_msgs::msg::CM2Ext& msg) {
  msg.cycleno = static_cast<uint32_t>(rel_cycle_num_);
  msg.time = rclcpp::Time(static_cast<int64_t>(1e9 * SimCore.Time), RCL_ROS_TIME);
  msg.synthdelay = synth_delay_;
  msg.header.stamp = rclcpp::Clock().now();
}

/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Add user specific Quantities for data storage
 *   and visualization to DataDictionary
 * - Called once at program start
 * - no realtime conditions
 *
 */
void CMNodeHelloCM::userDeclQuants(void) {

    tDDefault *df = DDefaultCreate("CMRosIF.");

    DDefULong   (df, "CycleNoRel",         "ms",  &rel_cycle_num_,              DVA_None);
    DDefInt     (df, "Sync.Cycles",        "-",   &CMNode.Sync.nCycles,         DVA_None);
    DDefDouble  (df, "Sync.Time",          "s",   &CMNode.Sync.Duration,        DVA_None);
    DDefDouble4 (df, "Sync.SynthDelay",    "s",   &synth_delay_,                DVA_IO_In);

    DDefUChar   (df, "Cfg.Mode",           "-",   (unsigned char*)&node_mode_,  DVA_None);
    DDefInt     (df, "Cfg.nCyclesClock",   "ms",  &clock_cycle_time_,           DVA_None);
    DDefChar    (df, "Cfg.SyncMode",       "-",   (char*)&sync_mode_,           DVA_None);
    DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",   &max_sync_time_,              DVA_IO_In);

    DDefaultDelete(df);
}

int CMNodeHelloCM::userTestrunStartAtBegin() {

  tErrorMsg *errv = nullptr;
  char sbuf[512];
  char *str         = nullptr;
  int idxC, idxS, idxP, ref;

  double def2c[]    = {0, 0};                // Default 2-col table data
  double def3c[]    = {0, 0, 0};             // Default 3-col table data

  /* Send transforms for coordinate systems */
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  tf2::Quaternion q;

  synth_delay_ = 1e-6;

  /* Prepare external node for next simulation */
  if (!srv_init_->wait_for_service(std::chrono::seconds(2))) {
    Log::printError(EC_Sim,
                    "ROS service is not ready! Please start external ROS node "
                    "providing service '" + std::string(srv_init_->get_service_name()) + "'!");
    node_mode_ = NodeMode::kDisabled;
    return -1;
  }

  Log::printLog("  -> Sending service request");
  srv_init_->async_send_request(
      std::make_shared<hellocm_msgs::srv::Init::Request>());
  
  { /* set up cm2ext publisher + job */
    typedef CMJob::RosPublisher<hellocm_msgs::msg::CM2Ext> cm2ext_t;
    auto job = std::make_shared<cm2ext_t>(nhp_, "cm2ext");
    job->setCycleTime(15000);
    job->registerCallback(&CMNodeHelloCM::cm2extFillMsg, this);
    scheduler_.addJob(job);
  }

  /* Read sensor info from Vehicle InfoFile */
  tInfos *Inf_Vehicle = nullptr;
  Inf_Vehicle = InfoNew();

  const char *FName;
  FName = InfoGetFilename(SimCore.Vhcl.Inf);

  int VehicleInfo_Err = iRead2(&errv, Inf_Vehicle, FName, "SensorReadCode");

  if (VehicleInfo_Err == 0) {

    CMNode.Misc.nSensors = iGetIntOpt(Inf_Vehicle, "Sensor.N", 0);

    { /* set up LidarRSI publisher + job */

      /* Read Sensor Infofile for LidarRSI */
      tInfos *Inf_Sensor = nullptr;
      tErrorMsg *errs = nullptr;

      Inf_Sensor = InfoNew();
      int SensorInfo_Err = iRead2(&errs, Inf_Sensor, "Data/Sensor/LidarRSI_FS_autonomous", "LidarCode");

      /* Extract LidarRSI Parameters from Sensor Infofile */
      if (SensorInfo_Err == 0) {
        CMNode.Sensor.LidarRSI.nBeams = iGetFixedTable2(Inf_Sensor, "Beams.N", 2, 1);
        CMNode.Sensor.LidarRSI.nBeams_h = CMNode.Sensor.LidarRSI.nBeams[0];
        CMNode.Sensor.LidarRSI.nBeams_v = CMNode.Sensor.LidarRSI.nBeams[1];
        CMNode.Sensor.LidarRSI.ret = iGetTableOpt(Inf_Sensor, "Beams", CMNode.Sensor.LidarRSI.BeamTable, tableSize, 6, &CMNode.Sensor.LidarRSI.rows);
        CMNode.Sensor.LidarRSI.FOV_h = iGetFixedTable2(Inf_Sensor, "Beams.FoVH", 2, 1); 
        CMNode.Sensor.LidarRSI.FOV_v = iGetFixedTable2(Inf_Sensor, "Beams.FoVV", 2, 1); 
        if (CMNode.Sensor.LidarRSI.ret == 0) {
          Log("Beam table read successfully. Table consists of %i rows\n", CMNode.Sensor.LidarRSI.rows);
          Log("FOVh = %f, FOVv = %f, nBeams_h = %f, nBeams_v = %f\n", 
            CMNode.Sensor.LidarRSI.FOV_h[0] + CMNode.Sensor.LidarRSI.FOV_h[1], 
            CMNode.Sensor.LidarRSI.FOV_v[0] + CMNode.Sensor.LidarRSI.FOV_v[1], 
            CMNode.Sensor.LidarRSI.nBeams_h, 
            CMNode.Sensor.LidarRSI.nBeams_v);
          Log("Beam Table --> First Azimuth Element = %f\n", CMNode.Sensor.LidarRSI.BeamTable[4*CMNode.Sensor.LidarRSI.rows]); 
          Log("Beam Table --> First Elevation Element = %f\n\n", CMNode.Sensor.LidarRSI.BeamTable[5*CMNode.Sensor.LidarRSI.rows]); 
        }
        CMNode.Sensor.LidarRSI.ret = InfoDelete(Inf_Sensor);
      } // if (SensorInfo_Err == 0)

      /* Find the first LidarRSI in the Vehicle Infofile */
      idxP = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        if (!strcmp(str, "LidarRSI")) {
          /* If the LidarRSI sensor is found, get its index and exit loop */
          idxP = ref;
          break;
        }
      }

      /* Store LidarRSI sensor parameters */
      if (idxP != -1) {
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        CMNode.Sensor.LidarRSI.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        CMNode.Sensor.LidarRSI.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        CMNode.Sensor.LidarRSI.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        CMNode.Sensor.LidarRSI.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 10);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        CMNode.Sensor.LidarRSI.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        /* Pass LidarRSI sensor parameters to job */
        q.setRPY(angles::from_degrees(CMNode.Sensor.LidarRSI.rot[0]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI.rot[1]), 
                 angles::from_degrees(CMNode.Sensor.LidarRSI.rot[2]));
        CMNode.TF.Lidar.transform.rotation = tf2::toMsg(q);
        CMNode.TF.Lidar.transform.translation.x = CMNode.Sensor.LidarRSI.pos[0];
        CMNode.TF.Lidar.transform.translation.y = CMNode.Sensor.LidarRSI.pos[1];
        CMNode.TF.Lidar.transform.translation.z = CMNode.Sensor.LidarRSI.pos[2];
        sprintf(sbuf, "Sensor.%d.name", idxS);
        CMNode.TF.Lidar.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIR00");
        sprintf(sbuf, "Sensor.%d.Mounting", idxS);
        CMNode.TF.Lidar.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
        transforms.push_back(CMNode.TF.Lidar);

        typedef CMJob::RosPublisher<sensor_msgs::msg::PointCloud> Lidar;
        auto job = std::make_shared<Lidar>(nhp_, "pointcloud");
        job->setCycleTime(CMNode.Sensor.LidarRSI.UpdRate);
        job->setCycleOffset(CMNode.Sensor.LidarRSI.nCycleOffset);
        job->registerCallback(&CMNodeHelloCM::pointcloudFillMsg, this);
        scheduler_.addJob(job);
      } else {
        CMNode.Sensor.LidarRSI.Active                   = 0;
      }
    } /* set up Lidar publisher + job */

    { /* set up ObjectList publisher + job */

      /* Find the first Object sensor in the Vehicle Infofile */
      idxP = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        if (!strcmp(str, "Object")) {
          /* If the Object sensor is found, get its index and exit loop */
          idxP = ref;
          break;
        }
      }

      /* Store Object sensor parameters */
      if (idxP != -1) {
        sprintf(sbuf, "Sensor.%d.Active", idxS);
        CMNode.Sensor.Object.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.%d.pos", idxS);
        CMNode.Sensor.Object.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.%d.rot", idxS);
        CMNode.Sensor.Object.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
        sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
        CMNode.Sensor.Object.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 100);
        sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
        CMNode.Sensor.Object.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

        /* Pass Object sensor parameters to job */
        q.setRPY(angles::from_degrees(CMNode.Sensor.Object.rot[0]), 
                 angles::from_degrees(CMNode.Sensor.Object.rot[1]), 
                 angles::from_degrees(CMNode.Sensor.Object.rot[2]));
        CMNode.TF.ObjectList.transform.rotation = tf2::toMsg(q);
        CMNode.TF.ObjectList.transform.translation.x = CMNode.Sensor.Object.pos[0];
        CMNode.TF.ObjectList.transform.translation.y = CMNode.Sensor.Object.pos[1];
        CMNode.TF.ObjectList.transform.translation.z = CMNode.Sensor.Object.pos[2];
        sprintf(sbuf, "Sensor.%d.name", idxS);
        CMNode.TF.ObjectList.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "OB00");
        sprintf(sbuf, "Sensor.%d.Mounting", idxS);
        CMNode.TF.ObjectList.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
        transforms.push_back(CMNode.TF.ObjectList); 

        typedef CMJob::RosPublisher<visualization_msgs::msg::MarkerArray> ObjectList;
        auto job = std::make_shared<ObjectList>(nhp_, "ObjectList");
        job->setCycleTime(1000.0/CMNode.Sensor.Object.UpdRate);
        job->setCycleOffset(CMNode.Sensor.Object.nCycleOffset);
        job->registerCallback(&CMNodeHelloCM::objectlistFillMsg, this);
        scheduler_.addJob(job);
      } else {
        CMNode.Sensor.Object.Active                   = 0;
      }
    } // set up ObjectList publisher + job

    { /* set up Camera publisher + job */
      
      /* Find the first Camera sensor in the Vehicle Infofile */
      idxP = -1;
        for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
          sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
          ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
          sprintf(sbuf, "Sensor.Param.%d.Type", ref);
          str = iGetStrOpt(Inf_Vehicle, sbuf, "");
          if (!strcmp(str, "Camera")) {
            /* If the Camera sensor is found, get its index and exit loop */
            idxP = ref;
            break;
          }
        }

        /* Store Camera sensor parameters */
        if (idxP != -1) {
          sprintf(sbuf, "Sensor.%d.Active", idxS);
          CMNode.Sensor.Camera.Active                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);
          sprintf(sbuf, "Sensor.%d.pos", idxS);
          CMNode.Sensor.Camera.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
          sprintf(sbuf, "Sensor.%d.rot", idxS);
          CMNode.Sensor.Camera.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
          sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
          CMNode.Sensor.Camera.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 100);
          sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
          CMNode.Sensor.Camera.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);

          /* Pass Camera sensor parameters to job */
          q.setRPY(angles::from_degrees(CMNode.Sensor.Camera.rot[0]), 
                   angles::from_degrees(CMNode.Sensor.Camera.rot[1]), 
                   angles::from_degrees(CMNode.Sensor.Camera.rot[2]));
          CMNode.TF.Camera.transform.rotation = tf2::toMsg(q);
          CMNode.TF.Camera.transform.translation.x = CMNode.Sensor.Camera.pos[0];
          CMNode.TF.Camera.transform.translation.y = CMNode.Sensor.Camera.pos[1];
          CMNode.TF.Camera.transform.translation.z = CMNode.Sensor.Camera.pos[2];
          sprintf(sbuf, "Sensor.%d.name", idxS);
          CMNode.TF.Camera.child_frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "LIN00");
          sprintf(sbuf, "Sensor.%d.Mounting", idxS);
          CMNode.TF.Camera.header.frame_id = iGetStrOpt(Inf_Vehicle, sbuf, "Fr1A");
          transforms.push_back(CMNode.TF.Camera);

          typedef CMJob::RosPublisher<camera_msgs::msg::CameraDetectionArray> Camera;
          auto job = std::make_shared<Camera>(nhp_, "Camera");
          job->setCycleTime(CMNode.Sensor.Camera.UpdRate);
          job->setCycleOffset(CMNode.Sensor.Camera.nCycleOffset);
          job->registerCallback(&CMNodeHelloCM::cameraFillMsg, this);
          scheduler_.addJob(job);
      } else {
        CMNode.Sensor.Camera.Active                   = 0;
      }
    } // set up Camera publisher + job

    { /* set up CameraRSI publishing nodes */

      int camera_no = 0;
      tCameraRSI Camera;

      /* Find each CameraRSI in the Vehicle Infofile */
      idxP = -1;
      for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++) {
        sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
        ref = iGetIntOpt(Inf_Vehicle, sbuf, 0);
        sprintf(sbuf, "Sensor.Param.%d.Type", ref);
        str = iGetStrOpt(Inf_Vehicle, sbuf, "");
        if (!strcmp(str, "CameraRSI")) {
          /* If the Camera sensor is found, get its index and extract its parameters */
          idxP = ref;
          sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
          idxC = iGetIntOpt(Inf_Vehicle, sbuf, 0);

          sprintf(sbuf, "Sensor.%d.Active", idxS);
          int camera_active = iGetIntOpt(Inf_Vehicle, sbuf, 0);

          /* Only store and process active camera details */
          if (camera_active) {
            /* Store Camera sensor parameters */
            sprintf(sbuf, "Sensor.%d.name", idxS);
            str                             = iGetStrOpt(Inf_Vehicle, sbuf, "");
            Camera.name = strdup(str);
            strcpy(Camera.name, str);
            Camera.number = camera_no;
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            Camera.pos                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            Camera.rot                      = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
            Camera.UpdRate                  = iGetIntOpt(Inf_Vehicle, sbuf, 100);
            sprintf(sbuf, "Sensor.Param.%d.FoV", idxP);
            Camera.FoV                      = iGetDblOpt(Inf_Vehicle, sbuf, 60);
            sprintf(sbuf, "Sensor.Param.%d.Resolution", idxP);
            Camera.Resolution               = iGetFixedTableOpt2(Inf_Vehicle, sbuf, def2c, 2, 1);
            sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
            Camera.nCycleOffset             = iGetIntOpt(Inf_Vehicle, sbuf, 0);
            sprintf(sbuf, "SensorCluster.%d.Socket", idxC);
            Camera.Socket                   = iGetIntOpt(Inf_Vehicle, sbuf, 0);

            /* Do a system call to ros2 launch and launch the ROS node */
            sprintf(sbuf, "ros2 launch carmaker_rsds_client carmaker_rsds_client.launch.py \
            camera_no:=%d \
            camera_name:=%s \
            rsds_port:=%d \
            param_trans_rot:=[%f,%f,%f,%f,%f,%f] \
            fov_deg:=%f \
            width:=%f \
            height:=%f &",
            Camera.number,
            Camera.name,
            Camera.Socket,
            Camera.pos[0],Camera.pos[1],Camera.pos[2],Camera.rot[0],Camera.rot[1],Camera.rot[2],
            Camera.FoV,
            Camera.Resolution[0],
            Camera.Resolution[1]);
            system(sbuf);

            CMNode.Sensor.CameraRSI.push_back(Camera);

            camera_no++;
          } // if (camera_active)
        } // if (!strcmp(str, "CameraRSI"))
      } // for (idxS = 0; idxS < CMNode.Misc.nSensors; idxS++)
    } // set up CameraRSI publishing nodes

    { /* set up VehicleControl subscriber + job */
      std::string topic = "VehicleControl";
      bool synchronize = (sync_mode_ == SyncMode::kTopic);
      CMJob::JobType job_type =
          synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

      typedef CMJob::RosSubscriber<vehiclecontrol_msgs::msg::VehicleControl> VC;
      auto job = std::make_shared<VC>(job_type, synchronize, nhp_, topic);
      job->setCycleTime(static_cast<unsigned long>(1));
      job->skipFirstCycles(1);
      job->registerCallback(&CMNodeHelloCM::vehicleControlCallback, this);
      scheduler_.addJob(job);
    } // set up VehicleControl subscriber + job

    { /* set up ext2cm subscriber + job */
      std::string topic = "ext2cm";
      bool synchronize = (sync_mode_ == SyncMode::kTopic);
      CMJob::JobType job_type =
          synchronize ? CMJob::JobType::Cyclic : CMJob::JobType::Trigger;

      auto cycle_time = param_client_->get_parameter("cycletime", 15000);

      typedef CMJob::RosSubscriber<hellocm_msgs::msg::Ext2CM> ext2cm_t;
      auto job = std::make_shared<ext2cm_t>(job_type, synchronize, nhp_, topic);
      job->setCycleTime(static_cast<unsigned long>(cycle_time));
      job->skipFirstCycles(1);
      job->setTimeoutTime(max_sync_time_);
      job->registerCallback(&CMNodeHelloCM::ext2cmCallback, this);
      scheduler_.addJob(job);

      if (cycle_time % clock_cycle_time_ != 0 ||
          (cycle_time < clock_cycle_time_ && clock_cycle_time_ > 0)) {
        node_mode_ = NodeMode::kDisabled;
        LogErrF(EC_Sim,
                "Ext. ROS node has an invalid cycle time! Expected multiple of "
                "%iums but got %ims",
                clock_cycle_time_, cycle_time);

        return -1;
      }
    } // set up ext2cm subscriber + job */
  } // if (VehicleInfo_Err == 0)

  InfoDelete(Inf_Vehicle);

  CMNode.TF.st_br->sendTransform(transforms);
  return 1;
}

int CMNodeHelloCM::userTestrunEnd() {

  /* Go through existing CameraRSI RSDS nodes and shut them down */
  if (!CMNode.Sensor.CameraRSI.empty()) {
    system("pkill -f carmaker_rsds_client_node > /dev/null 2>&1");
    CMNode.Sensor.CameraRSI.clear();
  }

  scheduler_.deleteJob("pointcloud");
  scheduler_.deleteJob("ObjectList");
  scheduler_.deleteJob("Camera");
  scheduler_.deleteJob("VehicleControl");
  scheduler_.deleteJob("ext2cm");
  scheduler_.deleteJob("cm2ext");
  return 1;
}

// Important: Use this macro to register the derived class as an interface with
// the CarMaker C++ Interface Loader module
REGISTER_CARMAKER_CPP_IF(CMNodeHelloCM)
