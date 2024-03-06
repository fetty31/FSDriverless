/*
******************************************************************************
**  CarMaker - Version 11.0.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Raw Signal Data Stream example client for IPGMovie 8.0.
**
** This example looks quite complex at first but is actually quite simple.
** - establish the RSDS connection: connect()
** - get the RSDS data: recv_hdr() and get_data()
** everything else has to do with saving the data or actualising the statistics
**
** Have a look at rsds-client-camera-basics.c for a much simpler example.
**
** Compiling:
** Linux
**	gcc -Wall -Os -o rsds-client-camera-standalone rsds-client-camera-standalone.c
** MS Windows (MSYS/MinGW)
**	gcc -Wall -Os -o rsds-client-camera-standalone rsds-client-camera-standalone.c -lws2_32
*/

#include <memory>
#include "rclcpp/create_timer.hpp"
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <angles/angles.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <signal.h>
#include <inttypes.h>
#include <unistd.h>

#ifdef WIN32
#  include <winsock2.h>
#else
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#endif

typedef enum {
    SaveFormat_DataNotSaved = 0,
    SaveFormat_Raw,
    SaveFormat_PPM,
    SaveFormat_PGM_byte,
    SaveFormat_PGM_short,
    SaveFormat_PGM_float
} tSaveFormat;

typedef struct {
    FILE *EmbeddedDataCollectionFile;
    std::string MovieHost;  /* pc on which IPGMovie runs          */
    int MoviePort;          /* TCP/IP port for RSDS               */
    int sock;               /* TCP/IP Socket                      */
    char sbuf[64];          /* Buffer for transmitted information */
    int RecvFlags;          /* Receive Flags                      */
    int Verbose;            /* Logging Output     			      */
    int ConnectionTries;
    tSaveFormat SaveFormat;
    int TerminationRequested;
    int Channel;
    int CameraNo;
} RSDScfg;

typedef struct {
    double tFirstDataTime;
    double tStartSim;
    double tEndSim;
    double tLastSimTime;
    unsigned long long int nBytesTotal;
    unsigned long long int nBytesSim;
    unsigned long int nImagesTotal;
    unsigned long int nImagesSim;
    unsigned char nChannels;
} RSDSIF;

using namespace std::placeholders;

class RSDS_Client : public rclcpp::Node {
 public:
  RSDS_Client() : Node("carmaker_rsds_client_node") {
    RCLCPP_INFO(this->get_logger(), "%s -> Start spinning...",
                rclcpp::Node::get_name());
    init();
  }

  ~RSDS_Client() {
    RCLCPP_INFO(this->get_logger(), "%s -> Shutdown", rclcpp::Node::get_name());
  }

 private:
  /*! Set up publisher and node */
  void init();

  /*! Scan TCP/IP Socket and writes to buffer */
  int recv_hdr(int sock, char *hdr);

  /* Connect over TCP/IP socket */
  int connect(void);

  /*! Data and image processing */
  int get_data(sensor_msgs::msg::Image::SharedPtr image, int *Channel);

  void print_node_info();
  void print_sim_info();
  void print_closing_info();

  inline double get_time();

  // Helpers for RSDSIF : RSDS information ( stats about current status)
  void add_data_to_stats(unsigned int len);
  void update_stats(unsigned int ImgLen, const char *ImgType, int Channel, int ImgWidth, int ImgHeight, float SimTime);
  void update_end_sim_time();
  // misc helpers
  void print_embedded_data (const char* data, unsigned int dataLen);

  /*! ROS node handle pointer */
  rclcpp::Node::SharedPtr nhp_;

  /*! Static transform broadcaster for the tf2 transform */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> st_br_;

  /*! RSDS interface parameters structure */
  RSDSIF rsdsif_;
  
  /*! RSDS configuration structure */
  RSDScfg rsdscfg_;

  /*! Image publisher*/
  image_transport::CameraPublisher image_pub_;

  /*! Image pixel stream */
  sensor_msgs::msg::Image::SharedPtr img_;

};

/*
 ** recv_hdr()
 **
 ** Scan TCP/IP Socket and writes to buffer
 */
int RSDS_Client::recv_hdr(int sock, char *hdr)
{
    const int HdrSize = 64;
    int len = 0;
    int nSkipped = 0;
    int i;

    while (1) {
        if (rsdscfg_.TerminationRequested)
            return -1;
        for (; len < HdrSize; len += i) {
            if ((i = recv(sock, hdr + len, HdrSize - len, rsdscfg_.RecvFlags)) <= 0) {
            	if (!rsdscfg_.TerminationRequested)
		            printf ("recv_hdr Error during recv (received: '%s' (%d))\n", hdr, len);
                return -1;
	        }
        }
        if (hdr[0] == '*' && hdr[1] >= 'A' && hdr[1] <= 'Z') {
            /* remove white spaces at end of line */
            while (len > 0 && hdr[len - 1] <= ' ')
                len--;
            hdr[len] = 0;
            if (rsdscfg_.Verbose == 1 && nSkipped > 0)
                printf("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
            return 0;
        }
        for (i = 1; i < len && hdr[i] != '*'; i++);
        len -= i;
        nSkipped += i;
        memmove(hdr, hdr + i, len);
    }
}

/*
 ** connect()
 **
 ** Connect over TCP/IP socket
 */
int RSDS_Client::connect(void)
{
#ifdef WIN32
    WSADATA WSAdata;
    if (WSAStartup(MAKEWORD(2,2), &WSAdata) != 0) {
        fprintf (stderr, "RSDS: WSAStartup ((2,2),0) => %d\n", WSAGetLastError());
        return -1;
    }
#endif

    struct sockaddr_in DestAddr;
    struct hostent *he;
    int tries = rsdscfg_.ConnectionTries;

    if ((he = gethostbyname(rsdscfg_.MovieHost.c_str())) == NULL) {
        RCLCPP_ERROR(this->get_logger(), "RSDS: unknown host: %s\n", rsdscfg_.MovieHost.c_str());
        return -2;
    }
    DestAddr.sin_family = AF_INET;
    DestAddr.sin_port = htons((unsigned short) rsdscfg_.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *) he->h_addr;
    rsdscfg_.sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    while (::connect(rsdscfg_.sock, (struct sockaddr *) &DestAddr, sizeof(DestAddr)) < 0 && tries > 0) {
        RCLCPP_INFO(this->get_logger(), "RSDS: can't connect '%s:%d'\n", rsdscfg_.MovieHost.c_str(), rsdscfg_.MoviePort);
        if (tries > 1) {
            RCLCPP_INFO(this->get_logger(), "\tretrying in 1 second... (%d)\n", --tries);
            sleep(1);
        } else {
            return -4;
        }
    }
    if (recv_hdr(rsdscfg_.sock, rsdscfg_.sbuf) < 0)
        return -3;

    printf("RSDS: Connected: %s\n", rsdscfg_.sbuf + 1);

    memset(rsdscfg_.sbuf, 0, 64);

    return 0;
}

/*
 ** get_data()
 **
 ** Data and image processing
 */
int RSDS_Client::get_data(sensor_msgs::msg::Image::SharedPtr image, int *Channel)
{
    unsigned int len = 0;
    ssize_t res = 0;

    /* Variables for Image Processing */
    char ImgType[32], AniMode[16];
    int ImgWidth, ImgHeight, ImgLen, dataLen;
    float SimTime;

    if (sscanf(rsdscfg_.sbuf, "*RSDS %d %s %f %dx%d %d", Channel, ImgType, &SimTime, &ImgWidth, &ImgHeight, &ImgLen) == 6) {

        update_stats(ImgLen, ImgType, *Channel, ImgWidth, ImgHeight, SimTime);

        if (rsdscfg_.Verbose == 1)
            RCLCPP_INFO(this->get_logger(), "[RSDS] %-6.3f : %-2d : %-8s %dx%d %d\n", SimTime, *Channel, ImgType, ImgWidth, ImgHeight, ImgLen);

        if (ImgLen > 0) {

            image->data.resize(ImgLen);
            // this is how we get the data
            for (int len = 0; len < ImgLen; len += res) {
                if ((res = recv(rsdscfg_.sock, &image->data[0] + len, ImgLen - len, rsdscfg_.RecvFlags)) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "RSDS: Socket Reading Failure\n");
                    break;
                }
            }

            const std::string encoding = static_cast<std::string>(ImgType);

            if (encoding == "rgb") {
                image->encoding = sensor_msgs::image_encodings::RGB8;
             } else if  (encoding == "grey") {
                image->encoding = sensor_msgs::image_encodings::MONO8;
             } else if  (encoding == "grey16") {
                image->encoding = sensor_msgs::image_encodings::MONO16;
             } else if  (encoding == "depth16") {
                image->encoding = sensor_msgs::image_encodings::MONO16;
             } else {
                RCLCPP_ERROR(this->get_logger(), "Incompatible image type/encoding: %s. Supported output formats: rgb, grey, grey16 and depth16.", ImgType);
             }

            image->width = static_cast<uint>(ImgWidth);
            image->height = static_cast<uint>(ImgHeight);
            image->step = image->width * static_cast<uint>(sensor_msgs::image_encodings::numChannels(image->encoding)
                                                 * 0.125 * sensor_msgs::image_encodings::bitDepth(image->encoding));
            image->is_bigendian = false;

            image->header.stamp = rclcpp::Time(static_cast<int64_t>(1e9 * SimTime), RCL_ROS_TIME);

            add_data_to_stats(len);
        }
        // needed for all channels, since we want the time until the last image
        update_end_sim_time();
    } else if (sscanf(rsdscfg_.sbuf, "*RSDSEmbeddedData %d %f %d %s", Channel, &SimTime, &dataLen, AniMode) == 4) {

        if (rsdscfg_.Verbose == 1)
            RCLCPP_INFO(this->get_logger(), "Embedded Data: %d %f %d %s\n", *Channel, SimTime, dataLen, AniMode);

        if (dataLen > 0) {
            char *data = (char *) malloc(dataLen);

            // get the data
            for (int len = 0; len < dataLen; len += res) {
                if ((res = recv(rsdscfg_.sock, data + len, dataLen - len, rsdscfg_.RecvFlags)) < 0) {
                    RCLCPP_ERROR(this->get_logger(), "RSDS: Socket Reading Failure\n");
                    free(data);
                    break;
                }
            }

            free(data);
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "RSDS: not handled: %s\n", rsdscfg_.sbuf);
    }

    return 0;
}

/*
 ** init()
 **
 ** Initialize Data Struct
 */
void RSDS_Client::init(void)
{

    rsdscfg_.MovieHost = "localhost";
    rsdscfg_.MoviePort = 2210;
    rsdscfg_.Verbose = 0;
    rsdscfg_.SaveFormat = SaveFormat_DataNotSaved;
    rsdscfg_.EmbeddedDataCollectionFile = NULL;
    rsdscfg_.RecvFlags = 0;
    rsdscfg_.ConnectionTries = 5;
    rsdscfg_.TerminationRequested = 0;
    rsdscfg_.Channel = 0;
    rsdscfg_.CameraNo = 0;

    rsdsif_.tFirstDataTime = 0.0;
    rsdsif_.tStartSim = 0.0;
    rsdsif_.tEndSim = 0.0;
    rsdsif_.tLastSimTime = -1.0;
    rsdsif_.nImagesSim = 0;
    rsdsif_.nImagesTotal = 0;
    rsdsif_.nBytesTotal = 0;
    rsdsif_.nBytesSim = 0;
    rsdsif_.nChannels = 0;

    int i, Channel;

    nhp_ = rclcpp::Node::make_shared(rclcpp::Node::get_name());

    /* Advertise the main image topic */
    image_transport::ImageTransport it(nhp_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // shared image message
    img_ = std::make_shared<sensor_msgs::msg::Image>();
    
    sensor_msgs::msg::CameraInfo::SharedPtr ci = std::make_shared<sensor_msgs::msg::CameraInfo>();

    st_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nhp_);

    /*
     ** The primary parameters here only serve to derive default lens and camera parameters.
     ** 
     ** The width and height are irrelevant for the actual image display as they get transmitted in the header of
     **  the RSDS data stream anyway.
     **
     ** A user that doesn't care about the lens parameters can simply supply (W,H) and sensible defaults will be produced.
     ** A user that cares abot lens parameters can ignore (W,H) and supply (K,R,P,D) coefficients directly.
     */

    /* Declare and obtain primary parameters */
    float width, height, fov_deg;
    std::vector<double> param_trans_rot;
    nhp_->declare_parameter<std::string>("carmaker_host", "localhost");
    nhp_->declare_parameter<std::string>("rsds_host", "localhost");
    nhp_->declare_parameter<int>("rsds_port", 2210);
    nhp_->declare_parameter<int>("connection_tries", 5);
    nhp_->declare_parameter<int>("camera_no", 0);
    nhp_->declare_parameter<std::string>("camera_frame", "CARS00");
    nhp_->declare_parameter("param_trans_rot", std::vector<double>{2.8, 0, 1.3, 0, 0, 0});
    nhp_->declare_parameter<float>("width", 1080);
    nhp_->declare_parameter<float>("height", 1920);
    nhp_->declare_parameter<float>("fov_deg", 60);
    
    nhp_->get_parameter("rsds_host", rsdscfg_.MovieHost);
    nhp_->get_parameter("rsds_port", rsdscfg_.MoviePort);
    nhp_->get_parameter("connection_tries", rsdscfg_.ConnectionTries);
    nhp_->get_parameter("camera_no", rsdscfg_.CameraNo);
    nhp_->get_parameter("camera_frame", img_->header.frame_id);
    nhp_->get_parameter("param_trans_rot", param_trans_rot);
    nhp_->get_parameter("width", width);
    nhp_->get_parameter("height", height);
    nhp_->get_parameter("fov_deg", fov_deg);    

    /* Declare and construct default camera name */
    std::string camera_name;
    nhp_->declare_parameter<std::string>("camera_name", "rsds_node_" + rsdscfg_.MovieHost + "_" + std::to_string(rsdscfg_.MoviePort));
    nhp_->get_parameter("camera_name", camera_name);

    /* Declare and obtain intermediate parameters (principal point) */
    float fov_rad, c_x, c_y;
    nhp_->declare_parameter<float>("fov_rad", fov_deg * 3.1415 / 180);
    nhp_->declare_parameter<float>("c_x", 0.5 * width);
    nhp_->declare_parameter<float>("c_y", 0.5 * height);

    nhp_->get_parameter("fov_rad", fov_rad);
    nhp_->get_parameter("c_x", c_x);
    nhp_->get_parameter("c_y", c_y);

    /* Declare and obtain intermediate parameters (focal length) */
    float f_x, f_y, max_f;
    nhp_->declare_parameter<float>("f_x", c_x/(tan(0.5 * fov_rad)));
    nhp_->declare_parameter<float>("f_y", c_y/(tan(0.5 * fov_rad)));
    nhp_->declare_parameter<float>("max_f", std::max(f_x, f_y));

    nhp_->get_parameter("f_x", f_x);
    nhp_->get_parameter("f_y", f_y);
    nhp_->get_parameter("max_f", max_f);

    /* Declare and obtain final parameters (lens coefficients) */
    std::vector<double> k_vec, r_vec, p_vec;
    nhp_->declare_parameter("calib_mat_k", std::vector<double>{max_f, 0, c_x, 0, max_f, c_y, 0, 0, 1});
    nhp_->declare_parameter("calib_mat_r", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    nhp_->declare_parameter("calib_mat_p", std::vector<double>{max_f, 0, c_x, 0, 0, max_f, c_y, 0, 0, 0, 1, 0});
    nhp_->declare_parameter("calib_mat_d", std::vector<double>{0});

    nhp_->get_parameter("calib_mat_k", k_vec);
    nhp_->get_parameter("calib_mat_r", r_vec);
    nhp_->get_parameter("calib_mat_p", p_vec);

    std::copy(k_vec.begin(), k_vec.begin() + 9, ci->k.begin());
    std::copy(r_vec.begin(), r_vec.begin() + 9, ci->r.begin());
    std::copy(p_vec.begin(), p_vec.begin() + 12, ci->p.begin());
    nhp_->get_parameter("calib_mat_d", ci->d);

    /* Declare parameters (binning, distortion) */
    nhp_->declare_parameter<int>("binning_x", 0);
    nhp_->declare_parameter<int>("binning_y", 0);
    nhp_->declare_parameter<std::string>("distortion", "plumb_bob");

    nhp_->get_parameter("binning_x", ci->binning_x);
    nhp_->get_parameter("binning_y", ci->binning_y);
    nhp_->get_parameter("distortion", ci->distortion_model);

    /* Create the transform for the camera frame of reference */
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = rclcpp::Time(0);
    tf.header.frame_id = "Fr1A";
    tf.child_frame_id = img_->header.frame_id;
    tf.transform.translation.x = param_trans_rot[0];
    tf.transform.translation.y = param_trans_rot[1];
    tf.transform.translation.z = param_trans_rot[2];
    tf2::Quaternion rot;
    tf2::Vector3 default_offset(-90, 0, -90);
    rot.setRPY(angles::from_degrees(param_trans_rot[3] + default_offset[0]),
               angles::from_degrees(param_trans_rot[4] + default_offset[1]),
               angles::from_degrees(param_trans_rot[5] + default_offset[2]));
    tf.transform.rotation = tf2::toMsg(rot);
    st_br_->sendTransform(tf);

    /* Connect to RSDS Server */
    if ((i = connect()) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Can't initialise RSDS Client (returns %d, %s)\n", i, i == -4 ? "No server": strerror(errno));
    }

    while (rclcpp::ok())
    {
        /* Read from TCP/IP-Port and process the image */
        if (rsdscfg_.TerminationRequested || recv_hdr(rsdscfg_.sock, rsdscfg_.sbuf) != 0) {
            break;
        }

        get_data(img_, &Channel);
        
        /* Grab the camera info */
        ci->header = img_->header;
        ci->height = img_->height;
        ci->width = img_->width;

        /* Publish the image if the image channel matches the desired channel from RSDScfg */
        if (Channel == rsdscfg_.Channel) { 
            RCLCPP_DEBUG(this->get_logger(), "Data received on CHANNEL: %-2d\n", Channel);
            image_pub_.publish(img_, ci);
        }
    }

    print_closing_info();

#ifdef WIN32
    closesocket (rsdscfg_.sock);
    WSACleanup();
#else
    close(rsdscfg_.sock);
#endif
}

void RSDS_Client::print_node_info()
{
    RCLCPP_INFO(this->get_logger(), "Node Parameters--------------------------\n");
    RCLCPP_INFO(this->get_logger(), "MovieHost:             %s\n", rsdscfg_.MovieHost);
    RCLCPP_INFO(this->get_logger(), "MoviePort:             %d\n", rsdscfg_.MoviePort);
    RCLCPP_INFO(this->get_logger(), "TerminationRequested:  %d\n", rsdscfg_.TerminationRequested);
    RCLCPP_INFO(this->get_logger(), "Channel:               %d\n", rsdscfg_.Channel);
}

void RSDS_Client::print_sim_info()
{
    double dtSimReal = rsdsif_.tEndSim - rsdsif_.tStartSim;
    // at least 1 sec of data is required
    if (dtSimReal > 1.0) {
        RCLCPP_INFO(this->get_logger(), "\nLast Simulation------------------\n");
        double MiBytes = rsdsif_.nBytesSim / (1024.0 * 1024.0);
        RCLCPP_INFO(this->get_logger(), "Duration: %.3f (real) %.3f (sim) -> x%.2f\n", dtSimReal, rsdsif_.tLastSimTime, rsdsif_.tLastSimTime / dtSimReal);
        RCLCPP_INFO(this->get_logger(), "Channels: %d\n", rsdsif_.nChannels);
        RCLCPP_INFO(this->get_logger(), "Images:   %ld (%.3f FPS)\n", rsdsif_.nImagesSim, rsdsif_.nImagesSim / dtSimReal);
        RCLCPP_INFO(this->get_logger(), "Bytes:    %.3f MiB (%.3f MiB/s)\n\n", MiBytes, MiBytes / dtSimReal);
    }
    if (rsdscfg_.EmbeddedDataCollectionFile != NULL)
        fflush(rsdscfg_.EmbeddedDataCollectionFile);

}

void RSDS_Client::print_closing_info()
{
    // from the very first image to the very last
    double dtSession = rsdsif_.tEndSim - rsdsif_.tFirstDataTime;
    RCLCPP_INFO(this->get_logger(), "\n-> Closing RSDS-Client...\n");

    // at least 1 sec of data is required
    if (dtSession > 1.0) {
        print_sim_info();
        RCLCPP_INFO(this->get_logger(), "Session--------------------------\n");
        double MiBytes = rsdsif_.nBytesTotal / (1024.0 * 1024.0);
        RCLCPP_INFO(this->get_logger(), "Duration: %g seconds\n", dtSession);
        RCLCPP_INFO(this->get_logger(), "Images:   %ld (%.3f FPS)\n", rsdsif_.nImagesTotal, rsdsif_.nImagesTotal / dtSession);
        RCLCPP_INFO(this->get_logger(), "Bytes:    %.3f MiB (%.3f MiB per second)\n", MiBytes, MiBytes / dtSession);
    }
    fflush(stdout);

    if (rsdscfg_.EmbeddedDataCollectionFile != NULL)
        fclose(rsdscfg_.EmbeddedDataCollectionFile);
}

// on a system with properly configured timers, calling this function should need less then 0.1us
inline double RSDS_Client::get_time()  // in seconds
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

void RSDS_Client::add_data_to_stats(unsigned int len)
{
    rsdsif_.nImagesTotal++;
    rsdsif_.nBytesTotal += len;
    rsdsif_.nImagesSim++;
    rsdsif_.nBytesSim += len;
}

void RSDS_Client::update_stats(unsigned int ImgLen, const char *ImgType, int Channel, int ImgWidth, int ImgHeight, float SimTime)
{
    if (rsdsif_.tFirstDataTime == 0.0)
        rsdsif_.tFirstDataTime = get_time();

    if (SimTime < 0.005 || rsdsif_.tLastSimTime < 0) {
        if (Channel == 0) {
            if (rsdsif_.tLastSimTime > 0)
                print_sim_info();
            RCLCPP_INFO(this->get_logger(), "-> Simulation started... (@ %.3f)\n", SimTime);
            rsdsif_.tStartSim = get_time();
            rsdsif_.nBytesSim = 0;
            rsdsif_.nImagesSim = 0;
            rsdsif_.nChannels = 1;
        }
        // this text will appear only for the first img of each channel
        if (rsdscfg_.Verbose == 2)
            RCLCPP_INFO(this->get_logger(), "%-6.3f : %-2d : %-8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);    }
    if (Channel == 0)
        rsdsif_.tLastSimTime = SimTime;

    if (Channel >= rsdsif_.nChannels)
        rsdsif_.nChannels = Channel + 1;
}

void RSDS_Client::update_end_sim_time()
{
    rsdsif_.tEndSim = get_time();
}

void RSDS_Client::print_embedded_data (const char* data, unsigned int dataLen)
{
    double * buf = (double *)data;
    unsigned int len =  dataLen/sizeof(double), i;
    for (i = 0; i < len; i++ ) {
        RCLCPP_INFO(this->get_logger(), "(%d) %f ", i, buf[i]);
    }
    RCLCPP_INFO(this->get_logger(), "\n");
}

int main(int argc, char* argv[]) {
  rclcpp::InitOptions options{};
  options.shutdown_on_sigint=true;
  rclcpp::init(argc, argv, options);
  rclcpp::spin(std::make_shared<RSDS_Client>());
  rclcpp::shutdown();
  return 0;
}