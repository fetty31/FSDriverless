/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef __LAPCOUNT_HH__
#define __LAPCOUNT_HH__

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <chrono>

class Lapcount{

    public:
        // Lapcount functions & variables
        bool Lapchanged = true;  // Lapcount flag
        bool start = false;      // Start flag

        Eigen::Vector3d old_position   = Eigen::Vector3d::Zero(); // Last position
        Eigen::Vector3d position       = Eigen::Vector3d::Zero(); // Current position
        Eigen::Vector2d first_position = Eigen::Vector2d::Zero(); // Current position

        //time variables
        std::chrono::_V2::system_clock::time_point tac;
        std::chrono::_V2::system_clock::time_point tic;

        std::chrono::duration<double> t; // Duration object for laptime

        float m, n;                  // Starting line slope & intercept
        float distMax = 3.0;         // Lapcount threshold distance

        double vx = 0.0;             // Car velocity
        double vx_min = 0.1;         // Minimum velocity to start lapcount

        int laps = 0;               // Number of laps

        // Returns the number of laps
        int getLaps(){
            return this->laps;
        }

        double getLapTime(){
            return this->t.count();
        }

        // Returns position relative to start/finish line
        float relativePosition(const Eigen::Vector3d point){
            return point(1)-m*point(0)-n;
        }

        // Get the new position and store the last one
        void setPosition(const Eigen::Vector3d point, double &vx){
            old_position = position;
            position     = point;
            this->vx     = vx;
        }

        // Sets start/finish line
        void getStartingLine(){
            float angle = position(2)+M_PI/2.0;

            m = tan(angle);
            n = position(1)-m*position(0);

            // save initial position
            this->first_position(0) = position(0);
            this->first_position(1) = position(1);
        }

        // Reset lapcount
        void reset(){
            this->laps = 0;
            this->Lapchanged = true;
            this->start = false;
            this->old_position   = Eigen::Vector3d::Zero();
            this->position       = Eigen::Vector3d::Zero();
            this->first_position = Eigen::Vector2d::Zero();
        }

        void run(){

            if(!this->start && vx >= vx_min){
                this->tac = std::chrono::system_clock::now();
                this->start = true;
            }

            double dist = sqrt(pow(position(0)-first_position(0),2)+pow(position(1)-first_position(1),2)); // Dist from the origin (0,0)

            // If we are close to the starting line (after having been far) and we cross the line, we have completed a lap
            if(dist <= distMax && !Lapchanged){

                if(relativePosition(old_position)*relativePosition(position)<=0){
                    laps++;
                    Lapchanged = !Lapchanged;

                    this->tic = std::chrono::system_clock::now();
                    this->t = tic-tac;
                    this->tac = tic;
                }

            // We indicate that we have been far from the origin 
            }else if(dist > distMax){
                Lapchanged = false;
            }
        }
};

#endif