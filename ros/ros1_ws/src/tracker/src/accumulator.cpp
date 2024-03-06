/*
 * @author Oriol Martínez @fetty31
 * @date 3-3-2024
 * @version 1.0
 *
 * Copyright (c) 2024 BCN eMotorsport
 */

#include "accumulator.hh"

namespace tracker{

Accumulator::Accumulator(double &mean, double &std) : 
        threshold(0.1), radius(20.0), 
        vision_angle(1.57), likelihood(0.9), 
        Lf(1.5), Lr(1.0), T(1.42) {
    noise = new noise::Gaussian(mean, std);
}

Accumulator::~Accumulator(){
    delete noise;
}

void Accumulator::create_KDTree(){ 
    std::vector<Point> vect;
    Point p;
    for(int i=0; i<track.size(); i++){
        p[0] = track[i].position[0];
        p[1] = track[i].position[1];
        vect.push_back(p);
    }
    Tree.build(vect);
    treeFlag = true; 

    seen_idx.reserve(track.size()); // reserve space for all seen cones
}

bool Accumulator::isTreeBuild(){
    return treeFlag;
}

void Accumulator::fillTracker(const visualization_msgs::MarkerArray::ConstPtr& msg){

    Point position;
    for(auto marker : msg->markers){
        position[0] = marker.pose.position.x;
        position[1] = marker.pose.position.y;
        noise->add_radial_noise(&position); // add noise to the cone position (x,y)

        int type = 0; // yellow cone by default
        if(marker.ns == "cone_blue") type = 1;
        else if(marker.ns == "cone_orange") type = 2;
        else if(marker.ns == "cone_orange_big") type = 3;

        Cone cone = Cone(marker.id, type, position, Point()); // we don't care about the cone's base link position for now
        this->track.push_back(cone);
    }
    this->create_KDTree();

}

template<typename NUM> NUM Accumulator::mod(NUM x, NUM y){ return sqrt(pow(x,2)+pow(y,2)); } // Norm of (x,y) vector

// Check if the given is the same cone as the one in the accumulator
bool Accumulator::sameCone(Point p){
    int nnid = Tree.nnSearch(p);
    if( mod<double>(p[0]-track[nnid].position[0], p[1]-track[nnid].position[1]) < this->threshold ) return true;
    else return false;
}

void Accumulator::fillConeMsg(int id, ipg_msgs::Cone* msg){
    msg->id = track[id].id;
    msg->type = track[id].type;
    msg->position_baseLink.x = track[id].positionBL[0];
    msg->position_baseLink.y = track[id].positionBL[1];
    msg->position_global.x = track[id].position[0];
    msg->position_global.y = track[id].position[1];
}

Point Accumulator::local2global(const Point p, const Eigen::Vector3d& pose) {
    Point point;
    point[0] = pose(0) + p[0]*cos(pose(2)) - p[1]*sin(pose(2));
    point[1] = pose(1) + p[0]*sin(pose(2)) + p[1]*cos(pose(2));
    return point;
}

Point Accumulator::global2local(const Point p, const Eigen::Vector3d& pose) {
    Point point;
    point[0] = +(p[0] - pose(0))*cos(pose(2)) + (p[1] - pose(1))*sin(pose(2));
    point[1] = -(p[0] - pose(0))*sin(pose(2)) + (p[1] - pose(1))*cos(pose(2));
    return point;
}

// Reset all values (a new Test Run is started)
bool Accumulator::reset(){

    // Clear (and reallocate) used vectors
    std::vector<Cone>().swap(track); 
    std::vector<int>().swap(seen_idx);

    // Clear set
    moved_idx.clear();

    // Reset flags
    this->treeFlag = false;
    this->stateFlag = false;

    // Reset cones down counter
    this->cones_down = 0;

    // Reset lapcount
    this->lapcount.reset();

    // Update reset flag
    this->resetFlag = true;

    return true;
}

// Check whether the cone is hit
bool Accumulator::coneDown(int &id, StateCar* state){

    if(track[id].positionBL[0] >= 0.0){ // front axle
        if( track[id].positionBL[0] < this->Lf && fabs(track[id].positionBL[1]) < this->T ){
            if(!this->track[id].down) cones_down++;
            track[id].down = true;
            if(this->moveConesFlag) this->moveCone(id, state); 
            return true;
        }
        return false;

    }else{ // rear axle
        if( fabs(track[id].positionBL[0]) < this->Lr && fabs(track[id].positionBL[1]) < this->T ){
            if(!this->track[id].down) cones_down++;
            track[id].down = true;
            if(this->moveConesFlag) this->moveCone(id, state); 
            return true;
        }
        return false;
    }
}

// Move hit cones
void Accumulator::moveCone(int &id, StateCar* state){
    track[id].position[0] += state->velocity(0) * cos(state->heading) * 0.025;
    track[id].position[1] += state->velocity(0) * sin(state->heading) * 0.025;
    track[id].positionBL = global2local(track[id].position, state->pose);
    track[id].moved = true;
    moved_idx.insert(id);
}

void Accumulator::fillConesDown(std_msgs::Float32MultiArray* msg){
    msg->data.clear();
    msg->layout.dim.clear();
    msg->layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg->layout.dim[0].size = moved_idx.size()*3;
    msg->layout.dim[0].stride = 3;
    msg->layout.dim[0].label  = (this->resetFlag) ? "reset" : "cones_down";
    this->resetFlag = false;

    std::set<int>::iterator it;
    for(it=moved_idx.begin(); it!=moved_idx.end(); ++it){
        msg->data.push_back(*it);
        msg->data.push_back(track[*it].position[0]);
        msg->data.push_back(track[*it].position[1]);
    }
}

int Accumulator::getNumConesDown(){
    return this->cones_down;
}

int Accumulator::getLaps(){
    return this->lapcount.getLaps();
}

// Get Laptime and Laps according to FSG rules
ipg_msgs::LapInfo Accumulator::getLapInfo(){
    ipg_msgs::LapInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.laps = this->lapcount.getLaps();
    msg.laptime = double(this->getNumConesDown())*2 + lapcount.getLapTime();
    return msg;
}

// Check if the cone is in the lidar field of view
bool Accumulator::isInLidarField(int id, StateCar* state){

    double r = mod<double>(track[id].position[0], track[id].position[1]);

    Point localPosLidar = global2local(track[id].position, state->lidar_pose);  // lidar frame pose 
    double phi = acos(localPosLidar[0]/r);
    if(phi < -M_PI) phi += 2*M_PI; // convert phi into [-180, 180]
    if(phi > M_PI) phi -= 2*M_PI;

    if(fabs(phi) < this->vision_angle && !track[id].seen){ // the cone is in our vision area (field) 
        if(noise::probability(this->likelihood)) return true; // likelihood of detecting the cone (percepcio pala)
        else return false;
    } else return false;

}

// Get all seen cones
void Accumulator::getSeenCones(StateCar* state, ipg_msgs::ConeArray* coneArray){

    ipg_msgs::Cone coneMsg = ipg_msgs::Cone();

    Point position;
    position[0] = state->pose(0);
    position[1] = state->pose(1);
    std::vector<int> coneIds = Tree.radiusSearch(position, this->radius); // search for cones

    // Check whether seen cones are inside LIDAR vision field
    for(auto id : coneIds){
        int idx = static_cast<int>(id);
        if(isInLidarField(idx, state)){
            track[idx].seen = true;
            this->seen_idx.push_back(idx);
        }
    }

    // Add all seen cones (with their current base_link position)
    for(auto idx : seen_idx){
        int cone_id = static_cast<int>(idx);
        track[cone_id].positionBL = global2local(track[cone_id].position, state->pose); // add current base_link position
        if(!coneDown(cone_id, state)){ // check for DOO (cone Down or Out) once BaseLink position is updated
            fillConeMsg(cone_id, &coneMsg);
            coneArray->cones.push_back(coneMsg);
        }
    }

    lapcount.setPosition(state->pose, state->velocity(0));
    if(!this->stateFlag) lapcount.getStartingLine();
    lapcount.run();

    this->stateFlag = true;
}

}