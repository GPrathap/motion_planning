#ifndef ALG_UTILS
#define ALG_UTILS

#include "data_type.h"
#include <functional>

typedef double huristics_cost_t;

template<typename State>
huristics_cost_t calculate_euclidean_dis(const State start_pt, const State end_pt)
{
    return std::sqrt(std::pow(start_pt->robot_state_[0] - end_pt->robot_state_[0], 2.0) + std::pow(start_pt->robot_state_[1]-end_pt->robot_state_[1], 2.0));
}

template<typename State>
huristics_cost_t calculate_manhattan_dis(const State start_pt, const State end_pt)
{           
    return (start_pt->robot_grid_index_.template cast<double>() - end_pt->robot_grid_index_.template cast<double>()).template lpNorm<1>();
}

template<typename State>
huristics_cost_t calculate_diagonal_dis(const State start_pt, const State end_pt)
{
    #define sqrt_3 1.7320508
    #define sqrt_2 1.4142136
    double dx = std::abs(start_pt->robot_grid_index_.x() - end_pt->robot_grid_index_.x());
    double dy = std::abs(start_pt->robot_grid_index_.y() - end_pt->robot_grid_index_.y());
    double dz = std::abs(start_pt->robot_grid_index_.z() - end_pt->robot_grid_index_.z());
    double min_xyz = std::min({dx, dy, dz});
    double max_xyz = std::max({dx, dy, dz});
    double mid_xyz = dx + dy + dz - min_xyz - max_xyz;
    return (sqrt_3 - sqrt_2) * min_xyz + (sqrt_2 - 1) * mid_xyz + max_xyz;
}

template<typename State>
huristics_cost_t calculate_dijkstra_dis(const State start_pt, const State end_pt)
{
   return 0;
}

template<typename State>
huristics_cost_t obvp(State start_ptr, State start_velocity, State target_ptr) {
    huristics_cost_t optimal_cost = std::numeric_limits<double>::max();
    double T = 0.0;
    auto start_position = start_ptr;
    auto target_position = target_ptr;
   
    return optimal_cost;
}



#endif
