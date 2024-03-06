/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ipg_msgs/CarState.h>
#include <string>

double steering=0.0;

void poseCallback(const ipg_msgs::CarState::ConstPtr& msg){
    steering = msg->steering;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "visualizer_tf");
    ros::NodeHandle nh("/carmaker");

    std::string carstate_topic;
    nh.param<std::string>("/IPG/Publishers/CarState/Topic", carstate_topic, "/carmaker/state");

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Quaternion q;

    ros::Subscriber subState = nh.subscribe(carstate_topic, 10, &poseCallback);

    ROS_WARN("WHEELS: waiting for base_link tf...");
    listener.waitForTransform("global", "base_link",
                              ros::Time::now(), ros::Duration(10.0)); // wait for global to base_link tf
    ROS_WARN("WHEELS: finished waiting!");

    ros::Rate rate(40.0);
    while (ros::ok()){
        try{
            
            // listener.waitForTransform("global", "base_link",
            //                   ros::Time::now(), ros::Duration(5.0)); // wait for global to base_link tf

            transform.setOrigin( tf::Vector3(0.8, -0.6, 0.25) );
            q.setRPY(0, 1.57079633, 1.57079633+steering);
            transform.setRotation(q);   
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "wheel_FR"));

            transform.setOrigin( tf::Vector3(0.8, 0.6, 0.25) );
            q.setRPY(0, 1.57079633, 1.57079633+steering);
            transform.setRotation(q); 
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "wheel_FL"));

            transform.setOrigin( tf::Vector3(-0.7, -0.6, 0.25) );
            q.setRPY(0, 1.57079633, 1.57079633);
            transform.setRotation(q); 
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "wheel_RR"));

            transform.setOrigin( tf::Vector3(-0.7, 0.6, 0.25) );
            q.setRPY(0, 1.57079633, 1.57079633);
            transform.setRotation(q); 
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "wheel_RL"));

        }catch(...){ }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}