/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef NOISE_CM_HH
#define NOISE_CM_HH

#include <random>
#include <cmath>
#include <chrono>

namespace noise{

class Gaussian{
    private:
        double mean;
        double std;
        double var;
    public:
        Gaussian(double mean, double std){
            this->mean = mean;
            this->std = std;
            this->var = std*std;
        }

        double get_noise(){
            std::normal_distribution<double> distribution(this->mean, this->var);

            // Construct a trivial random generator engine from a time-based seed:
            long seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            return distribution(generator);
        }

        double get_noise(double &u, double &lambda){
            std::normal_distribution<double> distribution(u, lambda);

            // Construct a trivial random generator engine from a time-based seed:
            long seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            return distribution(generator);
        }

        void add_noise(double *value){
            *value += this->get_noise();
        }

        void add_noise(double *value, double &mean, double &std){
            *value += this->get_noise(mean, std);
        }

        void add_radial_noise(double *point){
            double theta  = std::atan2(point[1], point[0]);
            double dr     = this->get_noise();
            double dtheta = this->get_noise();

            point[0] += std::cos(theta) * dr;
            point[1] += std::sin(theta) * dr;

            const double d = std::hypot(point[0], point[1]);

            theta += dtheta;

            point[0] = static_cast<float>(d * std::cos(theta));
            point[1] = static_cast<float>(d * std::sin(theta));
        }

        void reset(double mean, double std){
            this->mean = mean;
            this->std = std;
            this->var = std*std;
        }
};

}

#endif