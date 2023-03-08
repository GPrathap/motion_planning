#ifndef ROBOT_STATE_NODE
#define ROBOT_STATE_NODE


#include <Eigen/Dense>
#include <map>
#include <utility>
#include "data_type.h"
#include  <iostream>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <utility>
#include "backward.hpp"
#include "alg_utils.hpp"
#include "State.h"

struct RobotNode;


struct RobotNode {

    typedef RobotNode* Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum ROBOT_NODE_STATUS{
        WOULD_LIKE = 0, WILL_BE = 1, WAS_THERE = -1 
    };

    enum DIRECTION {
        FORWARD = 0, BACKWARD = 1, NO = 3
    };

    ROBOT_NODE_STATUS id_;
    Vec3f robot_state_;
    Vec3i robot_grid_index_;

    double g_score_{}, f_score_{};

    TrajectoryStatePtr trajectory_;//from parent to current
    Ptr parent_node_;

    explicit RobotNode(Vec3i robot_grid_index, Vec3f robot_state) {
        id_ = WOULD_LIKE;
        robot_grid_index_ = std::move(robot_grid_index);
        robot_state_ = std::move(robot_state);
        g_score_ = std::numeric_limits<double>::max();
        f_score_ = std::numeric_limits<double>::max();
        parent_node_ = nullptr;
        trajectory_ = nullptr;
    }

    explicit RobotNode(const Vec3i &grid_index) {
        id_ = WOULD_LIKE;
        robot_grid_index_ = grid_index;
        parent_node_ = nullptr;
    }

    void Reset() {
        id_ = WOULD_LIKE;
        parent_node_ = nullptr;
    }

    DIRECTION direction_{};
    vec_Vec3f intermediate_states_;
    int steering_grade_{};
};


struct RRTNode{
    typedef RRTNode* Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Vec3f coordinate_;
    RRTNode::Ptr parent_;
};


#endif //ROBOT_STATE_NODE