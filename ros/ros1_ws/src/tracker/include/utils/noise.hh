/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#ifndef NOISE_HH
#define NOISE_HH

#include <random>
#include <cmath>
#include <chrono>

#include <utils/structs.hh>

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

        void add_noise(Point* point){
            point->at(0) += this->get_noise();
            point->at(1) += this->get_noise();
        }

        void add_radial_noise(Point* point){
            double theta  = std::atan2(point->at(1), point->at(0));
            double dr     = this->get_noise();
            double dtheta = this->get_noise();

            point->at(0) += std::cos(theta) * dr;
            point->at(1) += std::sin(theta) * dr;

            const double d = std::hypot(point->at(0), point->at(1));

            theta += dtheta;

            point->at(0) = static_cast<float>(d * std::cos(theta));
            point->at(1) = static_cast<float>(d * std::sin(theta));
        }

        void reset(double mean, double std){
            this->mean = mean;
            this->std = std;
            this->var = std*std;
        }
};

inline bool probability(const double likelihood) {
    const double rand = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    return rand <= likelihood;
}

}

#endif