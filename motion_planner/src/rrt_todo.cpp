#include "rrt.h"
#include <ros/ros.h>
#include <random>
#include <iostream>

template<typename Graph, typename State>
void RRT<Graph, State>::setGraph(std::shared_ptr<Graph> graph){
    graph_ = graph;
}

template<typename Graph, typename State>
RRT<Graph, State>::~RRT() {
    delete end_node_;
    delete start_node_;
    start_node_ = nullptr;
    end_node_ = nullptr;
    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        delete nodes_ptr_[i];
        nodes_ptr_[i] = nullptr;
    }
    nodes_ptr_.clear();
}

template<typename Graph, typename State>
 typename State::Ptr RRT<Graph, State>::Near(const Eigen::Vector3d &pt) {
    std::vector<int> search_indices;
    std::vector<float> search_distances;
    pcl::PointXYZ point(pt.cast<float>().x(),pt.cast<float>().y(),pt.cast<float>().z());
    kdtree_flann_.nearestKSearch(point, 1, search_indices, search_distances);
    if (search_indices.empty()) {
        std::cerr << "kdtree is empty" << std::endl;
    }
    return nodes_ptr_[search_indices[0]];
}

template<typename Graph, typename State>
Eigen::Vector3d RRT<Graph, State>::Steer(const Eigen::Vector3d &rand_point,
                           const Eigen::Vector3d &near_point,
                           double step_size) {
    Eigen::Vector3d steered_point;               
    // TODO define your steer function
    return steered_point;
}

template<typename Graph, typename State>
pcl::PointCloud<pcl::PointXYZ> RRT<Graph, State>::GetNodesCoordPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    for (unsigned int i = 0; i < nodes_ptr_.size(); ++i) {
        pcl::PointXYZ point(nodes_ptr_[i]->coordinate_.template cast<float>().x(),
                            nodes_ptr_[i]->coordinate_.template cast<float>().y(),
                            nodes_ptr_[i]->coordinate_.template cast<float>().z());
        point_cloud.push_back(point);
    }

    return point_cloud;
}



template<typename Graph, typename State>
bool RRT<Graph, State>::SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
    const Eigen::Vector3d start = graph_->checkPointRange(start_pt);


    const Eigen::Vector3d end = graph_->checkPointRange(end_pt);

    end_node_ = new State();
    end_node_->coordinate_ = end;
    end_node_->parent_ = nullptr;

    start_node_ = new State();
    start_node_->coordinate_ = start;
    start_node_->parent_ = nullptr;
    nodes_ptr_.emplace_back(start_node_);

    pcl::PointCloud<pcl::PointXYZ> point_cloud_node = GetNodesCoordPointCloud();
    kdtree_flann_.setInputCloud(point_cloud_node.makeShared());

    ros::Time start_time = ros::Time::now();
    while (true) {
        // TODO generate sample point, use graph_->Sample();
        // TODO get near point (near_node_ptr) to the randomly generated point 
        typename State::Ptr near_node_ptr;
        // TODO Use the Steer function to get new point (new_point) 
        Eigen::Vector3d new_point;

        if (!graph_->collisionFree(near_node_ptr->coordinate_, new_point)) {
            continue;
        }

        typename State::Ptr new_node_ptr = new State();
        new_node_ptr->coordinate_ = new_point;
        new_node_ptr->parent_ = near_node_ptr;

        nodes_ptr_.push_back(new_node_ptr);

        if ((new_point - end_node_->coordinate_).norm() <= graph_->resolution_) {
            end_node_->parent_ = new_node_ptr;
            return true;
        }

        kdtree_flann_.setInputCloud(GetNodesCoordPointCloud().makeShared());

        ros::Duration use_time = ros::Time::now() - start_time;
        if (use_time.toSec() > 10.0){
            ROS_ERROR_STREAM("rrt has use time more than 10s ");
            return false;
        }
    }
}

template<typename Graph, typename State>
std::vector<Eigen::Vector3d> RRT<Graph, State>::GetPath() {
    std::vector<Eigen::Vector3d> path;
    typename State::Ptr node_ptr = end_node_;
    while (node_ptr != nullptr) {
        path.emplace_back(node_ptr->coordinate_);
        node_ptr = node_ptr->parent_;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

template class RRT<GridGraph3D, RRTNode>;