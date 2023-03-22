#ifndef ROBOT_PLANNER_INI
#define ROBOT_PLANNER_INI

#include <iostream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <iomanip>
#include "data_type.h"
#include "State.h"
#include "vis_utils.h"
#include "grid_graph.h"
#include <nav_msgs/OccupancyGrid.h>

#include "a_star.h"
#include "rrt.h"
#include "rrt_star.h"
#include "kino_a_star.h"
#include "kinematic_a_star.h"


class Planner {
    public:
        Planner() = default;
        ~Planner() = default;
        void init(ros::NodeHandle& nh);
        void waypointsCallback(const nav_msgs::Path &wp);
        void mapCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
        void setObstacles();
        void costMapCallBack(const nav_msgs::OccupancyGridPtr &costmap_msg_ptr);
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);
        void waypointCallback(const nav_msgs::PathConstPtr& msg);

        kino_planner::SearchInfo params;
        int max_iteration;
        
    private:
        double _map_resolution, _inv_map_resolution;
        double _x_size, _y_size, _z_size;
        Vec3f _start_pt, _start_velocity;
        bool _has_map = false;
        Vec3f _map_lower, _map_upper;
        int _max_x_id, _max_y_id, _max_z_id;
        ros::Subscriber _map_sub, _pts_sub, _cost_map_sub, _waypoint_sub, _odometry_sub;
        double _max_input_acc = 1.0;
        int _discretize_step = 2;
        double _time_interval = 1.25;
        int _time_step = 50;
        std::shared_ptr<VisualizationUtils> vis_util;
        std::shared_ptr<GridGraph3D> graph_;
        std::shared_ptr<AStar3D> _a_star;
        std::shared_ptr<RRT3D> _rrt;
        std::shared_ptr<RRTStar3D> _rrt_star;
        std::shared_ptr<KinoAStar3D> _kinodynamic_astar;
    
        
        nav_msgs::OccupancyGridPtr _cost_map_;
        std::deque<Eigen::Vector3d> waypoints_list;
        nav_msgs::Odometry odom;

};
#endif //ROBOT_PLANNER_INI