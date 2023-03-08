#include "a_star.h"
#include <cmath>
#include <unordered_set>

using namespace std;

template<typename Graph, typename State>
void AStar<Graph, State>::setGraph(std::shared_ptr<Graph> graph){
    graph_ = graph;
}


template<typename Graph, typename State>
void AStar<Graph, State>::breadthFirstSearch(const Vec3f &start_pt, const Vec3f &end_pt,
                     std::function<huristics_cost_t( typename State::Ptr a,  typename State::Ptr b)> calculate_huristics) {

    ros::Time start_time = ros::Time::now();
    Vec3i start_idx = graph_->coord2gridIndex(start_pt);
    Vec3i end_idx = graph_->coord2gridIndex(end_pt);

    typename State::Ptr current_node_ptr = nullptr;
    typename State::Ptr neighbor_node_ptr = nullptr;
    open_set_.clear();

    typename State::Ptr end_node_ptr = new State(end_idx, end_pt);
    end_node_ptr->id_ =  State::ROBOT_NODE_STATUS::WILL_BE;

    typename State::Ptr start_node_ptr = new State(start_idx, start_pt);
    start_node_ptr->id_ =  State::ROBOT_NODE_STATUS::WILL_BE;
    // TODO insert start_node_ptr into the open_set_
   

    std::vector< typename State::Ptr> neighbors_ptr;
    while (!open_set_.empty()) {
        current_node_ptr = open_set_.begin()->second;
        current_node_ptr->id_ = State::ROBOT_NODE_STATUS::WAS_THERE;
        // TODO remove current_node_ptr from the open_set_
        double dist = calculate_euclidean_dis<RobotNode::Ptr>(current_node_ptr, end_node_ptr);
        if (dist < graph_->resolution_) {
            terminate_ptr_ = current_node_ptr;
            ros::Duration use_time = ros::Time::now() - start_time;
            ROS_INFO("\033[1;32m Breah first search uses time: %f (ms)\033[0m", use_time.toSec() * 1000);
            return;
        }

        graph_->getNeighbors(current_node_ptr, neighbors_ptr);
        for (unsigned int i = 0; i < neighbors_ptr.size(); ++i) {
            neighbor_node_ptr = neighbors_ptr[i];
            if (neighbor_node_ptr->id_ == State::ROBOT_NODE_STATUS::WOULD_LIKE) {
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->id_ = State::ROBOT_NODE_STATUS::WILL_BE;
                open_set_.insert(std::make_pair(0, neighbor_node_ptr));
                continue;
            } 
        }
    }
    ROS_WARN_STREAM(" Breah first search failed to search path!");
}

template<typename Graph, typename State>
void AStar<Graph, State>::dijkstraSearchPath(const Vec3f &start_pt, const Vec3f &end_pt,
                     std::function<huristics_cost_t( typename State::Ptr a,  typename State::Ptr b)> calculate_huristics) {

    ros::Time start_time = ros::Time::now();
    Vec3i start_idx = graph_->coord2gridIndex(start_pt);
    Vec3i end_idx = graph_->coord2gridIndex(end_pt);

    
    ROS_WARN_STREAM(" Dijkstra failed to search path!");
}


template<typename Graph, typename State>
void AStar<Graph, State>::searchPath(const Vec3f &start_pt, const Vec3f &end_pt
                                , std::function<huristics_cost_t( typename State::Ptr a,  typename State::Ptr b)> calculate_huristics) {

}

template<typename Graph, typename State>
std::vector<Vec3f> AStar<Graph, State>::getPath() {
    std::vector<Vec3f> path;
    std::vector< typename State::Ptr> grid_path;
     typename State::Ptr grid_node_ptr = terminate_ptr_;
    while (grid_node_ptr != nullptr) {
        grid_path.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->parent_node_;
    }
    for (auto &ptr: grid_path) {
        path.emplace_back(ptr->robot_state_);
    }
    reverse(path.begin(), path.end());
    return path;
}

template class AStar<GridGraph3D, RobotNode>;