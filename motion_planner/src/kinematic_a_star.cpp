#include "kinematic_a_star.h"

template<typename Graph, typename State>
kinematicAStar<Graph, State>::kinematicAStar(kino_planner::SearchInfo& params, const int max_iterations) {
    segment_length_ = params.segment_length;
    steering_penalty_ = params.steering_penalty;
    steering_change_penalty_ = params.steering_change_penalty;
    reversing_penalty_ = params.reverse_penalty;
    shot_distance_ = params.shot_distance;
    max_iterations_ = max_iterations;
    rs_path_ptr_ = std::make_shared<RSPath>(params.wheel_base / std::tan(params.steering_angle * M_PI / 180.0));
    tie_breaker_ = 1.0 + 1e-3;
}

template<typename Graph, typename State>
kinematicAStar<Graph, State>::~kinematicAStar() {
    // ReleaseMemory();
}

template<typename Graph, typename State>
void kinematicAStar<Graph, State>::setGraph(std::shared_ptr<Graph> graph){
    graph_ = graph;
}


template<typename Graph, typename State>
double kinematicAStar<Graph, State>::ComputeH(const typename State::Ptr &current_node_ptr,
                             const typename State::Ptr &terminal_node_ptr) {
    double h;
    h = ((current_node_ptr->robot_state_.template cast<double>()).head(2) - (terminal_node_ptr->robot_state_.template cast<double>()).head(2)).template lpNorm<1>();

    if (h < 3.0 * shot_distance_) {
        h = rs_path_ptr_->Distance(current_node_ptr->robot_state_.x(), current_node_ptr->robot_state_.y(), current_node_ptr->robot_state_.z(),
                terminal_node_ptr->robot_state_.x(), terminal_node_ptr->robot_state_.y(), terminal_node_ptr->robot_state_.z());
    }
    return h;
}


template<typename Graph, typename State>
double kinematicAStar<Graph, State>::ComputeG(const typename State::Ptr &current_node_ptr,
                             const typename State::Ptr &neighbor_node_ptr) const {
    double g;
    if (neighbor_node_ptr->direction_ == State::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g;
}


template<typename Graph, typename State>
bool kinematicAStar<Graph, State>::Search(const Vec3d &start_state_world, const Vec3d &goal_state_world) {
    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_grid_index = graph_->State2IndexInit(start_state_world);
    const Vec3i goal_grid_index = graph_->State2IndexInit(goal_state_world);
    
    Vec3d start_state(start_grid_index[0], start_grid_index[1], 0.0);
    Vec3d goal_state(goal_grid_index[0], goal_grid_index[1], 0.0);

    // std::cerr<< "start: "<< start_grid_index.transpose() << std::endl;
    // std::cerr<< "target: "<< goal_grid_index.transpose() << std::endl;

    auto goal_node_ptr = new State(goal_grid_index);
    goal_node_ptr->robot_state_ = goal_state;
    goal_node_ptr->direction_ = State::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new State(start_grid_index);
    start_node_ptr->robot_state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = State::NO;
    start_node_ptr->id_ = State::WILL_BE;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_score_ = 0.0;
    start_node_ptr->f_score_ = ComputeH(start_node_ptr, goal_node_ptr);
    graph_->state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    graph_->state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<typename State::Ptr> neighbor_nodes_ptr;
    typename State::Ptr current_node_ptr;
    typename State::Ptr neighbor_node_ptr;
    int count = 0;
    while (!openset_.empty()) {
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->id_ = State::WAS_THERE;
        openset_.erase(openset_.begin());
        if ((current_node_ptr->robot_state_.head(2) - goal_node_ptr->robot_state_.head(2)).norm() <= shot_distance_) {
            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                terminal_node_ptr_ = goal_node_ptr;

                typename State::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;
                check_collision_use_time = 0.0;
                graph_->num_check_collision = 0.0;
                return true;
            }
        }


        graph_->GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            const Vec3i &index = neighbor_node_ptr->robot_grid_index_;
            if (graph_->state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->id_ = State::WILL_BE;
                neighbor_node_ptr->f_score_ = neighbor_node_ptr->g_score_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                graph_->state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (graph_->state_node_map_[index.x()][index.y()][index.z()]->id_ == State::WILL_BE) {
                double g_cost_temp = current_node_ptr->g_score_ + neighbor_edge_cost;

                if (graph_->state_node_map_[index.x()][index.y()][index.z()]->g_score_ > g_cost_temp) {
                    neighbor_node_ptr->g_score_ = g_cost_temp;
                    neighbor_node_ptr->f_score_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->id_ = State::WILL_BE;
                    graph_->state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                }
                continue;
            } else if (graph_->state_node_map_[index.x()][index.y()][index.z()]->id_ == State::WAS_THERE) {
                continue;
            }
        }
        count++;
        if (count > max_iterations_) {
            std::cerr<< ("Exceeded the number of iterations, the search failed")<< std::endl;
            return false;
     
        }
    }
    return false;
}

template<typename Graph, typename State>
void kinematicAStar<Graph, State>::ReleaseMemory() {
    terminal_node_ptr_ = nullptr;
}


template<typename Graph, typename State>
__attribute__((unused)) double kinematicAStar<Graph, State>::GetPathLength() const {
    return path_length_;
}


template<typename Graph, typename State>
std::vector<Eigen::Vector3d> kinematicAStar<Graph, State>::GetPath() const {
    std::vector<Eigen::Vector3d> path;

    std::vector<typename State::Ptr> temp_nodes;

    typename State::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    // std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(), node->intermediate_states_.end());
    }
    
    std::transform(path.cbegin(), path.cend(), path.begin(), [&](Eigen::Vector3d c ) {
        Vec3i index(c[0], c[1], c[2]); 
        auto pos = graph_->gridIndex2coord(index);
        return Eigen::Vector3d(pos[0], pos[1], c[2]);
    });


    return path;
}


template<typename Graph, typename State>
void kinematicAStar<Graph, State>::Reset() {
    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}


template<typename Graph, typename State>
bool kinematicAStar<Graph, State>::AnalyticExpansions(const typename State::Ptr &current_node_ptr,
                                     const typename State::Ptr &goal_node_ptr, double &length) {
    vec_Vec3f rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->robot_state_,
                                                        goal_node_ptr->robot_state_,
                                                        graph_->move_step_size_, length);
    for (const auto &pose: rs_path_poses)
        if (graph_->BeyondBoundary(pose.head(2)) || !graph_->CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);
    return true;
}

template class kinematicAStar<GridGraph3D, RobotNode>;