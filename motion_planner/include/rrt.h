//
// Created by meng on 2021/8/18.
//
#ifndef GRID_PATH_SEARCH_RRTSTAR_H
#define GRID_PATH_SEARCH_RRTSTAR_H

#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>
#include "data_type.h"
#include "grid_graph.h"
#include "robot_state.h"

template<typename Graph, typename State>
class RRT{
public:
    RRT() = default;
    virtual ~RRT();
    virtual bool SearchPath(const Vec3f &start_pt, const Vec3f &end_pt);
    std::vector<Eigen::Vector3d> GetPath();
    virtual void setGraph(std::shared_ptr<Graph> graph);
   

protected:
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_flann_;
    std::vector<typename State::Ptr> nodes_ptr_;
    typename State::Ptr end_node_;
    typename State::Ptr start_node_;
    typename State::Ptr Near(const Eigen::Vector3d& pt);
    Eigen::Vector3d Steer(const Eigen::Vector3d& rand_point, const Eigen::Vector3d& near_point, double step_size);
    pcl::PointCloud<pcl::PointXYZ>GetNodesCoordPointCloud();
    std::shared_ptr<Graph> graph_;
};

typedef RRT<GridGraph3D, RRTNode> RRT3D;
#endif //GRID_PATH_SEARCH_RRTSTAR_H
