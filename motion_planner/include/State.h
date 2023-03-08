#ifndef _STATE_H_
#define _STATE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <utility>
#include "backward.hpp"
#include "data_type.h"

struct TrajectoryState;
typedef TrajectoryState *TrajectoryStatePtr;

struct TrajectoryState {
    std::vector<Vec3f> Position;
    std::vector<Vec3f> Velocity;
    double Trajctory_Cost;
    bool collision_check;           //False -> no collision, True -> collision
    bool optimal_flag;               //False -> not optimal in traj_library_, True -> optimal in traj_library_,

    TrajectoryState(std::vector<Vec3f> _Position, std::vector<Vec3f> _Velocity,
                    double _Trajctory_Cost) {
        Position = std::move(_Position);
        Velocity = std::move(_Velocity);
        Trajctory_Cost = _Trajctory_Cost;
        collision_check = false;
        optimal_flag = false;
    }

    TrajectoryState() {};

    ~TrajectoryState() {};

    void setCollisionfree() {
        collision_check = true;
    }

    void setOptimal() {
        optimal_flag = true;
    }
};

#endif