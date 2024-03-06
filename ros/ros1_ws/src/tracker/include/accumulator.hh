/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef TRACKER_ACCUM_HH
#define TRACKER_ACCUM_HH

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

#include <ipg_msgs/Cone.h>
#include <ipg_msgs/ConeArray.h>
#include <ipg_msgs/LapInfoArray.h>
#include <ipg_msgs/CarState.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <cmath>
#include <set>
// #include <unordered_map>

#include <utils/kdtree.h>
#include <utils/lapcount.hh>
#include <utils/noise.hh>
#include <utils/structs.hh>

namespace tracker{

class Accumulator{
  public:
    kdt::KDTree<Point> Tree;
    // std::unordered_map<int, Cone> conemap;
    std::vector<Cone> track;
    std::vector<int> seen_idx;
    std::set<int> moved_idx;

    Lapcount lapcount;
    noise::Gaussian* noise;

    double threshold, radius, vision_angle;
    double likelihood;
    double Lf, Lr, T; // length and track of the car
    bool treeFlag = false, stateFlag = false, resetFlag = false, moveConesFlag = false;
    int cones_down = 0; 

    Accumulator(double &mean, double &std);
    ~Accumulator();

    void create_KDTree();

    void fillTracker(const visualization_msgs::MarkerArray::ConstPtr& msg);

    template<typename NUM> NUM mod(NUM x, NUM y); // Norm of (x,y) vector

    bool sameCone(Point p); // Check if given cone is the same as the one in the accumulator

    void fillConeMsg(int id, ipg_msgs::Cone* msg);

    Point local2global(const Point p, const Eigen::Vector3d& pose);
    Point global2local(const Point p, const Eigen::Vector3d& pose);

    bool reset(); // Reset all values (a new Test Run is started)
    
    bool coneDown(int &id, StateCar* state); // Check whether the cone is hit

    void moveCone(int &id, StateCar* state); // Move hit cones

    void fillConesDown(std_msgs::Float32MultiArray* msg);

    int getNumConesDown();
    int getLaps();

    bool isTreeBuild();
    
    ipg_msgs::LapInfo getLapInfo(); // Get Laptime and Laps according to FSG rules
    
    bool isInLidarField(int id, StateCar* state); // Check if the cone is in the lidar field of view

    void getSeenCones(StateCar* state, ipg_msgs::ConeArray* coneArray); // Get all seen cones

};

typedef std::unique_ptr<Accumulator> AccumPtr;

}

#endif