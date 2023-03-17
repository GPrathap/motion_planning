//
// Created by meng on 2021/8/26.
//

#ifndef GRID_PATH_SEARCHER_RRT_STAR_H
#define GRID_PATH_SEARCHER_RRT_STAR_H

#include "rrt.h"

template<typename Graph, typename State>
class RRTStar : public RRT<Graph, State> {
public:
    RRTStar() = default;

    bool SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) override;

    ~RRTStar() override;

private:
    std::vector< typename State::Ptr> NearC(const Eigen::Vector3d &new_point, double search_radius);

    void chooseParent(const std::vector< typename State::Ptr> &near_nodes, typename  State::Ptr const &new_node);

    double getPathLength(const typename  State::Ptr start_node_ptr, const typename  State::Ptr end_node_ptr);

    void rewire(std::vector< typename State::Ptr> &near_nodes, const  typename State::Ptr& new_node);
};

typedef RRTStar<GridGraph3D, RRTNode> RRTStar3D;
#endif //GRID_PATH_SEARCHER_RRT_STAR_H
