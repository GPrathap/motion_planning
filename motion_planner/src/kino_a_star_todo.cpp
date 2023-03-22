#include "kino_a_star.h"
#include <cmath>
#include <unordered_set>

using namespace std;


template<typename Graph, typename State>
void KinoAStar<Graph, State>::setGraph(std::shared_ptr<Graph>& graph){
    graph_ = graph;
}


template<typename Graph, typename State>
void KinoAStar<Graph, State>::searchPath(const Vec3f &start_pt, const Vec3f &end_pt, std::function<huristics_cost_t(Vec3f a, Vec3f start_velocity, Vec3f b)> calculate_huristics) {
    ros::Time start_time = ros::Time::now();
    Vec3i start_idx = graph_->coord2gridIndex(start_pt);
    Vec3f start_velocity(0.0, 0.0, 0.0);

     typename State::Ptr current_node_ptr = nullptr;
     typename State::Ptr neighbor_node_ptr = nullptr;
    open_set_.clear();

    typename State::Ptr start_node_ptr = new State(start_idx, start_pt);
    start_node_ptr->g_score_ = 0.0;
    start_node_ptr->f_score_ = calculate_huristics(start_pt, start_velocity, end_pt);
    start_node_ptr->id_ = State::WOULD_LIKE;
    open_set_.insert(std::make_pair(start_node_ptr->f_score_, start_node_ptr));

    std::vector< typename State::Ptr> neighbors_ptr;
    std::vector<MotionStatePtr> neighbors_traj_state;

    while (!open_set_.empty()) {
        current_node_ptr = open_set_.begin()->second;
        current_node_ptr->id_ = State::WAS_THERE;
        open_set_.erase(open_set_.begin());

        double dist = (current_node_ptr->robot_state_ - end_pt).norm();
        if (dist < graph_->resolution_) {
            terminate_ptr_ = current_node_ptr;
            ros::Duration use_time = ros::Time::now() - start_time;
            ROS_INFO("\033[1;32m Kinodynamic A* uses time: %f (ms)\033[0m", use_time.toSec() * 1000);
            return;
        }

        MotionStateMapPtr motion_state_ptr;
        if (current_node_ptr->trajectory_ == nullptr) {
            motion_state_ptr = motionPrimitiveSet(current_node_ptr->robot_state_, start_velocity, end_pt, calculate_huristics);
        } else {
            motion_state_ptr = motionPrimitiveSet(current_node_ptr->robot_state_, current_node_ptr->trajectory_->Velocity.back(), end_pt, calculate_huristics);
        }

        graph_->getNeighbors(motion_state_ptr, neighbors_ptr, neighbors_traj_state, max_allowed_steps_);

        for (unsigned int i = 0; i < neighbors_ptr.size(); ++i) {
            neighbor_node_ptr = neighbors_ptr[i];
            double delta_score = (neighbor_node_ptr->robot_state_ - current_node_ptr->robot_state_).norm();
            if (neighbor_node_ptr->id_ == State::WOULD_LIKE) {
                neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + delta_score;
                neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + neighbors_traj_state[i]->MotionState_Cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->trajectory_ = neighbors_traj_state[i];
                open_set_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                neighbor_node_ptr->id_ = State::WILL_BE;
                continue;
            } else if (neighbor_node_ptr->id_ == State::WILL_BE) {
                if (neighbor_node_ptr->g_score_ > current_node_ptr->g_score_ + delta_score) {
                    neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + delta_score;
                    neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + neighbors_traj_state[i]->MotionState_Cost;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    delete neighbor_node_ptr->trajectory_;
                    neighbor_node_ptr->trajectory_ = neighbors_traj_state[i];

                    typename std::multimap<double,  typename State::Ptr>::iterator map_iter = open_set_.begin();
                    for (; map_iter != open_set_.end(); map_iter++) {
                        if (map_iter->second->robot_grid_index_ == neighbor_node_ptr->robot_grid_index_) {
                            open_set_.erase(map_iter);
                            open_set_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                            break;
                        }
                    }
                }
                continue;
            } else {
                continue;
            }
        }
    }
    ROS_WARN_STREAM("Hybrid A* failed to search path!");
}


template<typename Graph, typename State>
MotionStateMapPtr KinoAStar<Graph, State>::motionPrimitiveSet(const Vec3f &start_pt, const Vec3f &start_velocity, const Vec3f &target_pt
                                                    , std::function<huristics_cost_t(Vec3f a, Vec3f start_velocity, Vec3f b)> calculate_huristics_) {
    Vector3d acc_input;
    Vector3d pos, vel;
    int a = 0;
    int b = 0;
    int c = 0;

    double min_Cost = 100000.0;
    double MotionState_Cost;
    MotionStateMapPtr motionPrimitives;
    motionPrimitives = new MotionStatePtr **[max_allowed_steps_ + 1];
    for (int i = 0; i <= max_allowed_steps_; i++) {
        motionPrimitives[i] = new MotionStatePtr *[max_allowed_steps_ + 1];
        for (int j = 0; j <= max_allowed_steps_; j++) {
            motionPrimitives[i][j] = new MotionStatePtr[max_allowed_steps_ + 1];
            for (int k = 0; k <= max_allowed_steps_; k++) {
                vector<Vector3d> Position;
                vector<Vector3d> Velocity;

                // TODO calculate acc_input 
               
                // TODO append start position and velocity into Position and Velocity, respectively. 
                
                bool collision = false;
                double delta_time;
                // TODO estimate delta_time, you may use time_interval_ and time_step_ values appropriately 
                
                // TODO estimate trajectory, i.e., pos, vel, over each from the current pos to time_step_ times based on acceleration (acc_input)
                MotionState_Cost = calculate_huristics_(pos, vel, target_pt);
                motionPrimitives[i][j][k] = new MotionState(Position, Velocity, MotionState_Cost);
                // TODO estimate minimum cost index, i.e., a, b, c
            }
        }
    }
    motionPrimitives[a][b][c]->setOptimal();
    return motionPrimitives;
}


template<typename Graph, typename State>
std::vector<Vec3f> KinoAStar<Graph, State>::getPath() {
    std::vector<Vec3f> path;
    std::vector< typename State::Ptr> grid_path;
    typename State::Ptr grid_node_ptr = terminate_ptr_;
    while (grid_node_ptr != nullptr) {
        grid_path.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->parent_node_;
    }
    for (auto &ptr: grid_path) {
        if (ptr->trajectory_ == nullptr) {
            continue;
        }
        for (auto iter = ptr->trajectory_->Position.end() - 1; iter != ptr->trajectory_->Position.begin(); iter--) {
            path.emplace_back(*iter);
        }
    }
    reverse(path.begin(), path.end());
    return path;
}

template class KinoAStar<GridGraph3D, RobotNode>;