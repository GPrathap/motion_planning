#include "planner.h"

void Planner::init(ros::NodeHandle& nh){

    params.steering_angle = nh.param("planner/steering_angle", 10);
    params.steering_angle_discrete_num = nh.param("planner/steering_angle_discrete_num", 1);
    params.wheel_base = nh.param("planner/wheel_base", 1.0);
    params.segment_length = nh.param("planner/segment_length", 1.6);
    params.segment_length_discrete_num = nh.param("planner/segment_length_discrete_num", 8);
    params.steering_penalty = nh.param("planner/steering_penalty", 1.05);
    params.steering_change_penalty = nh.param("planner/steering_change_penalty", 1.5);
    params.reverse_penalty = nh.param("planner/reversing_penalty", 2.0);
    params.shot_distance = nh.param("planner/shot_distance", 5.0);
    params.track = nh.param("planner/track", 1.3);
    params.vehicle_width = nh.param("planner/vehicle_width", 0.5);
    params.vehicle_length = nh.param("planner/vehicle_length", 1.0);
    params.grid_size_phi = nh.param("planner/grid_size_phi", 72);


    vis_util = std::make_shared<VisualizationUtils>();
    graph_ = std::make_shared<GridGraph3D>();
    _a_star = std::make_shared<AStar3D>();
    _rrt = std::make_shared<RRT3D>();
    _rrt_star = std::make_shared<RRTStar3D>();

    max_iteration = 10000;
    _map_sub = nh.subscribe("map", 1, &Planner::mapCallBack, this);
    _pts_sub = nh.subscribe("waypoints", 1, &Planner::waypointsCallback, this);
    _cost_map_sub = nh.subscribe("/projected_map", 1, &Planner::costMapCallBack, this);
    _waypoint_sub = nh.subscribe("/planner/waypoints", 1, &Planner::waypointCallback, this);
    _odometry_sub = nh.subscribe<nav_msgs::Odometry>("/odom_world", 50, &Planner::odomCallback, this);

    nh.param("map/resolution", _map_resolution, 0.2);

    nh.param("map/x_size", _x_size, 50.0);
    nh.param("map/y_size", _y_size, 50.0);
    nh.param("map/z_size", _z_size, 5.0);

    nh.param("planning/start_x", _start_pt(0), 0.0);
    nh.param("planning/start_y", _start_pt(1), 0.0);
    nh.param("planning/start_z", _start_pt(2), 0.0);

    nh.param("planning/start_vx", _start_velocity(0), 0.0);
    nh.param("planning/start_vy", _start_velocity(1), 0.0);
    nh.param("planning/start_vz", _start_velocity(2), 0.0);

    _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
    _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;

    _inv_map_resolution = 1.0 / _map_resolution;

    _max_x_id = (int) (_x_size * _inv_map_resolution);
    _max_y_id = (int) (_y_size * _inv_map_resolution);
    _max_z_id = (int) (_z_size * _inv_map_resolution);

    vis_util->init(nh, _map_resolution, _discretize_step);
    graph_->InitGridMap(_map_lower, _map_upper, Vector3i(_max_x_id, _max_y_id, _max_z_id), _map_resolution);
    graph_->InitGridMap(_map_resolution, _max_x_id, _max_y_id, params);

    _a_star->setGraph(graph_);

     _rrt->setGraph(graph_);
    _rrt_star->setGraph(graph_);
}

void Planner::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom = *msg;
}

void Planner::waypointCallback(const nav_msgs::PathConstPtr& msg)
{

  std::cout<< "Waypoints are bring called " << std::endl;

  waypoints_list.clear();
  if (msg->poses.size() < 0.0){
    std::cout<< "empty waypoints are detected.." << std::endl;
    return;
  }

  std::cout << "Triggered!" << std::endl;
  for(int i=0; i<(int)msg->poses.size(); i++){
    Eigen::Vector3d wp(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
    std::cout<< wp.transpose() << std::endl;
    waypoints_list.push_back(wp);
  }
}

void Planner::costMapCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr){
    std::cout<< "getting cost map" << std::endl;
    _cost_map_ = costmap_msg_ptr;
}

void Planner::waypointsCallback(const nav_msgs::Path &wp) {
    if (wp.poses[0].pose.position.z < 0.0 || !_has_map)
        return;

    Vec3f target_pt;
    target_pt << wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, 0.0;
    
    std::function<huristics_cost_t(RobotNode::Ptr,  RobotNode::Ptr)> heuristic = &calculate_dijkstra_dis<RobotNode::Ptr>;
    graph_->reset();
    _a_star->setGraph(graph_);
    _a_star->dijkstraSearchPath(_start_pt, target_pt, heuristic);
    auto path_dijkstra = _a_star->getPath();
    std::cout<< "path length: "<< path_dijkstra.size() << std::endl;
    vis_util->visDijkstraPath(path_dijkstra);

    graph_->reset();
    _a_star->setGraph(graph_);
    _a_star->breadthFirstSearch(_start_pt, target_pt, heuristic);
    auto path_breath_first_search = _a_star->getPath();
    vis_util->visBreathFirstSearchPath(path_breath_first_search);
    
    graph_->reset();
    _a_star->setGraph(graph_);
    heuristic = &calculate_euclidean_dis<RobotNode::Ptr>;
    _a_star->searchPath(_start_pt, target_pt, heuristic);
    auto path_astar = _a_star->getPath();
    vis_util->visAStarPath(path_astar);

    // ros::Time start  = ros::Time::now();
    // graph_->reset();
    // _rrt->setGraph(graph_);
    
    // if (_rrt->SearchPath(_start_pt, target_pt)) {
    //     ros::Time end = ros::Time::now();
    //     ros::Duration rrt_use_time = end - start;
    //     ROS_INFO_STREAM("rrt path finding use time: " << rrt_use_time.toSec());
    //     auto path_rrt = _rrt->GetPath();
    //     vis_util->visRRTPath(path_rrt);
    // }

    // ros::Time start_rrt_star = ros::Time::now();
    // graph_->reset();
    // _rrt_star->setGraph(graph_);
    // if (_rrt_star->SearchPath(_start_pt, target_pt)) {
    //     ros::Time end_rrt_star = ros::Time::now();
    //     ros::Duration rrt_star_use_time = end_rrt_star - start_rrt_star;
    //     ROS_INFO_STREAM("rrt star path finding use time: " << rrt_star_use_time.toSec());
    //     auto path_rrt_star = _rrt_star->GetPath();
    //     vis_util->visRRTstarPath(path_rrt_star);
    // }  
}

void Planner::setObstacles(){
    unsigned int map_w = std::floor(_cost_map_->info.width / _map_resolution);
    unsigned int map_h = std::floor(_cost_map_->info.height / _map_resolution);
    for (unsigned int w = 0; w < map_w; ++w) {
        for (unsigned int h = 0; h < map_h; ++h) {
            auto x = static_cast<unsigned int> ((w + 0.5) * _map_resolution/ _cost_map_->info.resolution);
            auto y = static_cast<unsigned int> ((h + 0.5) * _map_resolution/ _cost_map_->info.resolution);
            if (_cost_map_->data[y * _cost_map_->info.width + x] == 100) {
                graph_->SetObstacle(std::make_unsigned_t<int>(w), std::make_unsigned_t<int>(h));
            }
        }
    }
}


void Planner::mapCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
    if (_has_map) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    pcl::fromROSMsg(pointcloud_map, cloud);
    if ((int) cloud.points.size() == 0) return;
    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int) cloud.points.size(); idx++) {
        pt = cloud.points[idx];
        graph_->setObs(Vec3f(pt.x, pt.y, pt.z));
        Vec3f cor_round = graph_->coordRounding(Vec3f(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }
    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;
    vis_util->visPointCloud(cloud_vis);
    _has_map = true;
}
