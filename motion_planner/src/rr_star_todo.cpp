#include "rrt_star.h"
#include <ros/ros.h>

template<typename Graph, typename State>
RRTStar<Graph, State>::~RRTStar() {
    delete this->end_node_;
    delete this->start_node_;
    this->start_node_ = nullptr;
    this->end_node_ = nullptr;

    for (unsigned int i = 0; i < this->nodes_ptr_.size(); ++i) {
        delete this->nodes_ptr_[i];
        this->nodes_ptr_[i] = nullptr;
    }
    this->nodes_ptr_.clear();
}

template<typename Graph, typename State>
bool RRTStar<Graph, State>::SearchPath(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt) {
    const Eigen::Vector3d start = this->graph_->checkPointRange(start_pt);
    const Eigen::Vector3d end =  this->graph_->checkPointRange(end_pt);

    this->end_node_ = new State();
    this->end_node_->coordinate_ = end;
    this->end_node_->parent_ = nullptr;

    this->start_node_ = new  State();
    this->start_node_->coordinate_ = start;
    this->start_node_->parent_ = nullptr;
    this->nodes_ptr_.emplace_back(this->start_node_);

    pcl::PointCloud<pcl::PointXYZ> point_cloud_node = this->GetNodesCoordPointCloud();
    this->kdtree_flann_.setInputCloud(point_cloud_node.makeShared());

    ros::Time start_time = ros::Time::now();
    while (true) {
        Eigen::Vector3d rand_point =  this->graph_->Sample();
        typename State::Ptr near_node_ptr = this->Near(rand_point);

        Eigen::Vector3d new_point = this->Steer(rand_point, near_node_ptr->coordinate_, this->graph_->resolution_ * 2);

        if (!this->graph_->collisionFree(near_node_ptr->coordinate_, new_point)) {
            continue;
        }

        typename State::Ptr new_node_ptr = new State();
        new_node_ptr->coordinate_ = new_point;

        std::vector< typename State::Ptr > near_nodes;
        // TODO Get near nodes (near_nodes) relative to the new_point 
        chooseParent(near_nodes, new_node_ptr);
        // TODO Rewire the near_nodes from the new node 
        rewire(near_nodes, new_node_ptr);

        if ((new_point - this->end_node_->coordinate_).norm() <= this->graph_->resolution_) {
            new_node_ptr->parent_ = near_node_ptr;
            this->end_node_->parent_ = new_node_ptr;
            return true;
        }

        this->nodes_ptr_.push_back(new_node_ptr);

        this->kdtree_flann_.setInputCloud(this->GetNodesCoordPointCloud().makeShared());

        ros::Duration use_time = ros::Time::now() - start_time;
        if (use_time.toSec() > 10.0){
            ROS_ERROR_STREAM("rrt star has use time more than 10s ");
            return false;
        }
    }
}

template<typename Graph, typename State>
std::vector< typename State::Ptr > RRTStar<Graph, State>::NearC(const Eigen::Vector3d &new_point, const double search_radius) {
    std::vector<int> search_indices;
    std::vector<float> search_distances;
    pcl::PointXYZ point_xyz(new_point.cast<float>().x(),
                            new_point.cast<float>().y(),
                            new_point.cast<float>().z());
    this->kdtree_flann_.radiusSearch(point_xyz, search_radius, search_indices, search_distances);

    std::vector< typename State::Ptr > near_nodes;
    near_nodes.reserve(search_indices.size());
    for (unsigned int i = 0; i < search_indices.size(); ++i) {
        near_nodes.emplace_back(this->nodes_ptr_[search_indices[i]]);
    }

    return near_nodes;
}

template<typename Graph, typename State>
void RRTStar<Graph, State>::chooseParent(const std::vector< typename State::Ptr > &near_nodes, typename State::Ptr const &new_node) {
    double min_length = std::numeric_limits<double>::max();
    typename State::Ptr parent_node = nullptr;
    for (unsigned int i = 0; i < near_nodes.size(); ++i) {
        if (!this->graph_->collisionFree(near_nodes[i]->coordinate_, new_node->coordinate_))
            continue;

        new_node->parent_ = near_nodes[i];

        double length = getPathLength(this->start_node_, new_node);
        if (length < min_length) {
            min_length = length;
            parent_node = near_nodes[i];
        }
    }

    new_node->parent_ = parent_node;
}

template<typename Graph, typename State>
void RRTStar<Graph, State>::rewire(std::vector< typename State::Ptr > &near_nodes, const typename State::Ptr &new_node) {
    for (unsigned int i = 0; i < near_nodes.size(); ++i) {
        if (!this->graph_->collisionFree(near_nodes[i]->coordinate_, new_node->coordinate_))
            continue;

        double dist_before = getPathLength(this->start_node_, near_nodes[i]);
        double dist_after = getPathLength(this->start_node_, new_node) +
                            (new_node->coordinate_ - near_nodes[i]->coordinate_).norm();

        if (dist_after < dist_before) {
            // TODO set the near node parent as the new node 
            near_nodes[i]->parent_;
        }
    }
}

template<typename Graph, typename State>
double RRTStar<Graph, State>::getPathLength(const typename State::Ptr start_node_ptr, const typename State::Ptr end_node_ptr) {
    double length = 0.0;

    typename State::Ptr temp_node_ptr = end_node_ptr;
    while (temp_node_ptr->parent_ != nullptr && temp_node_ptr != start_node_ptr) {
        length += (temp_node_ptr->coordinate_ - temp_node_ptr->parent_->coordinate_).norm();
        temp_node_ptr = temp_node_ptr->parent_;
    }

    return length;
}


template class RRTStar<GridGraph3D, RRTNode>;