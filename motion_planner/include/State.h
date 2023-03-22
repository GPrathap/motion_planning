#ifndef _STATE_H_
#define _STATE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <utility>
#include "backward.hpp"
#include "data_type.h"

struct MotionState;
typedef MotionState *MotionStatePtr;
typedef MotionStatePtr ***MotionStateMapPtr;

struct MotionState {
    std::vector<Vec3f> Position;
    std::vector<Vec3f> Velocity;
    double MotionState_Cost;
    bool collision_check; 
    bool optimal_flag;

    MotionState(std::vector<Vec3f> _Position, std::vector<Vec3f> _Velocity,
                    double _motion_cost) {
        Position = std::move(_Position);
        Velocity = std::move(_Velocity);
        MotionState_Cost = _motion_cost;
        collision_check = false;
        optimal_flag = false;
    }

    MotionState() {};

    ~MotionState() {};

    void setCollisionfree() {
        collision_check = true;
    }

    void setOptimal() {
        optimal_flag = true;
    }
};

#endif