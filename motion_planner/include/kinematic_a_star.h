#ifndef kinematic_A_STAR_kinematic_A_STAR_H
#define kinematic_A_STAR_kinematic_A_STAR_H

#include "rs_paths.h"
// #include "state_node.h"
#include "robot_state.h"
#include "grid_graph.h"
#include <glog/logging.h>
#include "data_type.h"
#include <map>
#include <memory>
#include "types.hpp"
#include <algorithm>


template<typename Graph, typename State>
class kinematicAStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    kinematicAStar() = delete;

    kinematicAStar(kino_planner::SearchInfo& params, const int max_iterations);
    ~kinematicAStar();

    bool Search(const Vec3d &start_state, const Vec3d &goal_state);
    void setGraph(std::shared_ptr<Graph> graph);
    std::vector<Eigen::Vector3d> GetPath() const;
    
    __attribute__((unused)) double GetPathLength() const;
    void Reset();

private:

    bool AnalyticExpansions(const typename State::Ptr &current_node_ptr,
                            const typename State::Ptr &goal_node_ptr, double &length);

    inline double ComputeG(const typename State::Ptr &current_node_ptr, const typename State::Ptr &neighbor_node_ptr) const;

    inline double ComputeH(const typename State::Ptr &current_node_ptr, const typename State::Ptr &terminal_node_ptr);

    void ReleaseMemory();

private:
    
    typename State::Ptr terminal_node_ptr_ = nullptr;
    std::multimap<double, typename State::Ptr> openset_;
    double segment_length_;
    double tie_breaker_;
    double shot_distance_;
    double steering_penalty_;
    double reversing_penalty_;
    double steering_change_penalty_;
    double theta_splt = 72;
    double path_length_ = 0.0;
    std::shared_ptr<RSPath> rs_path_ptr_;
    int max_iterations_ = 5000;
    double check_collision_use_time = 0.0;
    std::shared_ptr<Graph> graph_;
    
};


///Planner for 3D VoxelMap
typedef kinematicAStar<GridGraph3D, RobotNode> kinematicAStar2D;
#endif //kinematic_A_STAR_kinematic_A_STAR_H
