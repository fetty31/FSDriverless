/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef STRUCTS_HH
#define STRUCTS_HH

#include <eigen3/Eigen/Dense>

// This class is for the KDTree
class Point : public std::array<double,2> {
    public :
        static const int DIM = 2;
};

struct StateCar {
    Eigen::Vector3d pose;
    Eigen::Vector3d lidar_pose;
    Eigen::Vector3d velocity;
    double heading;
    StateCar() : pose(Eigen::Vector3d(0,0,0)), lidar_pose(Eigen::Vector3d(0,0,0)), velocity(Eigen::Vector3d(0,0,0)), heading(0.0) {}
};

struct Cone {

    uint32_t id;
    uint8_t type; 
    Point position;     // global position
    Point positionBL;   // base link position
    bool seen;          // seen flag just to know if this cone was already seen
    bool down;          // whether we have hit this cone or not (DOO)
    bool moved;         // whether the cone has been moved after hitting it

    Cone(uint32_t Id, uint8_t Type, Point Position , Point PositionBL ) : id(Id), type(Type), position(Position),
                                                                             positionBL(PositionBL), seen(false), 
                                                                             down(false), moved(false) {} 
};

#endif