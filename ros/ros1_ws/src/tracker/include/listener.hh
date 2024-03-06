/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef TRACKER_LISTENER_HH
#define TRACKER_LISTENER_HH

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <cmath>
#include <set>

namespace tracker{

class Listener{
    public:
        std::string target_frame; // get TF to this frame
        std::string source_frame; // get TF from this frame 
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener tfListener;
        geometry_msgs::Vector3 position;
        double roll, pitch, yaw;

        Listener(std::string child_frame, std::string parent_frame) : tfListener(buffer), 
                target_frame(child_frame), 
                source_frame(parent_frame), 
                position(geometry_msgs::Vector3()) {}

        void get_transform(){
            geometry_msgs::TransformStamped transformStamped;
            try{
                transformStamped = buffer.lookupTransform(target_frame, source_frame, /* TF from source to target */
                                        ros::Time(0), ros::Duration(2.0));
                position.x = transformStamped.transform.translation.x; 
                position.y = transformStamped.transform.translation.y; 
                position.z = transformStamped.transform.translation.z;

                tf2::Quaternion q(
                        transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);
                tf2::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);

                // ROS_ERROR_STREAM("LIDAR position x: " << position.x);
                // ROS_ERROR_STREAM("LIDAR position y: " << position.y);
                // ROS_ERROR_STREAM("LIDAR position z: " << position.z);
                // ROS_ERROR_STREAM("LIDAR yaw angle: " << yaw);
                
            }catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                // ros::Duration(1).sleep();
            }
        }
};

typedef std::unique_ptr<Listener> ListenerPtr;

}

#endif