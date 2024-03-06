/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
*/

#include <ros/ros.h>
#include <accumulator.hh>
#include <listener.hh>

#include <tracker/Init.h>

#include <string>

ros::Publisher pubCones, pubDown, pubLap, pubMoved;

tracker::ListenerPtr listen_ptr;
tracker::AccumPtr accum_ptr;

StateCar state;
ipg_msgs::LapInfoArray lapInfoArray;
std_msgs::Float32MultiArray movedArray;
int laps = 0;

void state_callback(const ipg_msgs::CarState::ConstPtr& msg){

    // Global pose
    state.pose(0) = msg->odom.position.x;
    state.pose(1) = msg->odom.position.y;
    state.pose(2) = msg->odom.heading;

    // Global velocity
    state.velocity(0) = msg->odom.velocity.x;
    state.velocity(1) = msg->odom.velocity.y;
    state.velocity(2) = msg->odom.velocity.w;

    // Heading
    state.heading = msg->odom.heading;

    // Call TF transform
    listen_ptr->get_transform();
    state.lidar_pose(0) = listen_ptr->position.x;
    state.lidar_pose(1) = listen_ptr->position.y;
    state.lidar_pose(2) = listen_ptr->yaw;

    // Publish Seen Cones
    if(accum_ptr->isTreeBuild()){
        ipg_msgs::ConeArray coneArrayMsg = ipg_msgs::ConeArray();
        accum_ptr->getSeenCones(&state, &coneArrayMsg);
        pubCones.publish(coneArrayMsg);

        std_msgs::Int32 conesdown_msg;
        conesdown_msg.data = accum_ptr->getNumConesDown();
        pubDown.publish(conesdown_msg);
    }

    // Publish Lap Info
    if(accum_ptr->getLaps()>laps){
        lapInfoArray.info.push_back(accum_ptr->getLapInfo());
        laps++;
    }
    pubLap.publish(lapInfoArray);
}

void track_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){

    if(!accum_ptr->isTreeBuild()) accum_ptr->fillTracker(msg);
    
    // Publish moved cones new positions
    accum_ptr->fillConesDown(&movedArray);
    pubMoved.publish(movedArray);

}

bool reset(tracker::Init::Request  &req, tracker::Init::Response &res){
    
    res.success = accum_ptr->reset(); // Reset accumulator obj
    lapInfoArray = ipg_msgs::LapInfoArray(); // Reset lap info array
    laps = 0; // Reset lap counter

    ROS_WARN("TRACKER: Reseting accumulator...");
    return true;
    
}

int main(int argc, char **argv) {

    // Init node
    ros::init(argc, argv, "tracker");

    // Handle connections
    ros::NodeHandle nh("~");

    std::string stateTopic, trackTopic, conesTopic, downTopic, lapTopic, movedTopic, resetTopic;
    // Input topics
    nh.param<std::string>("Topics/Input/State", stateTopic, "/carmaker/state");
    nh.param<std::string>("Topics/Input/Track", trackTopic, "/carmaker/track");

    // Output topics
    nh.param<std::string>("Topics/Output/Cones",        conesTopic,     "/carmaker/cones");
    nh.param<std::string>("Topics/Output/ConesDown",    downTopic,      "/carmaker/conesdown");
    nh.param<std::string>("Topics/Output/Laptime",      lapTopic,       "/carmaker/lapinfo");
    nh.param<std::string>("Topics/Output/Moved",        movedTopic,     "/carmaker/moved");

    nh.param<std::string>("Service/Reset", resetTopic, "/tracker/reset");

    // Subscribers & Publishers
    ros::Subscriber subState = nh.subscribe(stateTopic, 10, &state_callback);
    ros::Subscriber subTrack = nh.subscribe(trackTopic, 10, &track_callback);
    
    pubCones = nh.advertise<ipg_msgs::ConeArray>(conesTopic, 10);
    pubDown  = nh.advertise<std_msgs::Int32>(downTopic, 10);
    pubLap   = nh.advertise<ipg_msgs::LapInfoArray>(lapTopic, 10);
    pubMoved = nh.advertise<std_msgs::Float32MultiArray>(movedTopic, 10);

    // Reset server
    ros::ServiceServer service = nh.advertiseService(resetTopic, &reset);

    double noise_mean, noise_std;
    nh.param<double>("Noise/Mean", noise_mean, 0.0);
    nh.param<double>("Noise/Std",  noise_std,  0.0);

    // Call tracker constructors
    listen_ptr = std::unique_ptr<tracker::Listener>(new tracker::Listener("global", "lidar"));
    accum_ptr = std::unique_ptr<tracker::Accumulator>(new tracker::Accumulator(noise_mean, noise_std));

    double track;
    nh.param<double>("BoundingBox/Lf", accum_ptr->Lf, 1.5); 
    nh.param<double>("BoundingBox/Lr", accum_ptr->Lr, 1.0); 
    nh.param<double>("BoundingBox/T", track, 1.4); 
    accum_ptr->T = track/2.0; // we take half the track

    nh.param<bool>("ConesMoving", accum_ptr->moveConesFlag, false);


    double angle;
    nh.param<double>("Detection/Radius",        accum_ptr->radius,      20.0);
    nh.param<double>("Detection/Likelihood",    accum_ptr->likelihood,  0.9);
    nh.param<double>("Detection/VisionAngle",   angle,                  180.0);
    accum_ptr->vision_angle = angle * M_PI / 360.0;
    /*NOTE: we convert the vision angle from degrees to rad. We also divide it by 2
        because the vision field is divided into 2 by the x-axis */ 

    nh.param<double>("Lapcount/MinVel", accum_ptr->lapcount.vx_min, 0.1);

    ros::spin(); // keep spinning

    return 0;
}