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
huristics_cost_t min_acc_obvp(State start_ptr, State start_velocity, State target_ptr) {
    huristics_cost_t optimal_cost = std::numeric_limits<double>::max();
    double T = 0.0;
    auto start_position = start_ptr;
    auto target_position = target_ptr;
    
    double alpha_1, alpha_2, alpha_3, beta_1, beta_2, beta_3;
    const double px_0 = start_position.x();
    const double py_0 = start_position.y();
    const double pz_0 = start_position.z();
    const double vx_0 = start_velocity.x();
    const double vy_0 = start_velocity.y();
    const double vz_0 = start_velocity.z();

    const double px_T = target_position.x();
    const double py_T = target_position.y();
    const double pz_T = target_position.z();
    const double vx_T = 0.0;
    const double vy_T = 0.0;
    const double vz_T = 0.0;

    double a, b, c, d;

    a = 0.0;
    b = -4 * (std::pow(vx_0, 2) + std::pow(vy_0, 2) + std::pow(vz_0, 2));
    c = -24 * (px_0 * vx_0 - px_T * vx_0 + py_0 * vy_0 - py_T * vy_0 + pz_0 * vz_0 - pz_T * vz_0);
    d = 72 * (pz_0 * pz_T + py_0 * py_T + px_0 * px_T) - 36 * (std::pow(pz_T, 2) + std::pow(pz_0, 2) + std::pow(py_T, 2)+ std::pow(py_0, 2) + std::pow(px_T, 2) + std::pow(px_0, 2));

    Eigen::Matrix4d A;
    A << 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -d, -c, -b, -a;

    Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(A);
    Eigen::MatrixXd r;
    r = eigen_solver.eigenvalues().real();
    for (int i = 0; i < 4; ++i) {
        if (r(i) > T) {
            T = r(i);
        }
    }

    alpha_1 = -12.0 * (px_T - vx_0 * T - px_0) / std::pow(T, 3) + 6.0 * (vx_T - vx_0) / std::pow(T, 2);
    alpha_2 = -12.0 * (py_T - vy_0 * T - py_0) / std::pow(T, 3) + 6.0 * (vy_T - vy_0) / std::pow(T, 2);
    alpha_3 = -12.0 * (pz_T - vz_0 * T - pz_0) / std::pow(T, 3) + 6.0 * (vz_T - vz_0) / std::pow(T, 2);
    beta_1 = 6.0 * (px_T - vx_0 * T - px_0) / std::pow(T, 2) - 2.0 * (vx_T - vx_0) / T;
    beta_2 = 6.0 * (py_T - vy_0 * T - py_0) / std::pow(T, 2) - 2.0 * (vy_T - vy_0) / T;
    beta_3 = 6.0 * (pz_T - vz_0 * T - pz_0) / std::pow(T, 2) - 2.0 * (vz_T - vz_0) / T;

    optimal_cost = T + 1.0 / 3.0 * std::pow(alpha_1, 2) * std::pow(T, 3) + alpha_1 * beta_1 * std::pow(T, 2) + std::pow(beta_1, 2) * T
                   + 1.0 / 3.0 * std::pow(alpha_2, 2) * std::pow(T, 3) + alpha_2 * beta_2 * std::pow(T, 2) + std::pow(beta_2, 2) * T
                   + 1.0 / 3.0 * std::pow(alpha_3, 2) * std::pow(T, 3) + alpha_3 * beta_3 * std::pow(T, 2) + std::pow(beta_3, 2) * T;
    return optimal_cost;

}



#endif
