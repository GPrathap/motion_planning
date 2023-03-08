#include "vis_utils.h"


void VisualizationUtils::init(ros::NodeHandle& nh, double resolution_map, double _discretize_step_){
     _grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_libvis", 1);
    _kino_a_star_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("kino_a_star_path_vis", 1);
    _a_star_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("a_star_path_vis", 1);
    _dijkstra_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("dijkstra_path_vis", 1);
    _breah_first_search_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("breah_first_search_vis", 1);
    _RRTstar_path_vis_pub = nh.advertise<visualization_msgs::Marker>("rrt_star_path_vis", 1);
    _RRT_path_vis_pub = nh.advertise<visualization_msgs::Marker>("rrt_path_vis", 1);
    _kinematic_a_star_pub = nh.advertise<visualization_msgs::MarkerArray>("kinematic_a_star_path_vis", 1);
     path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 1);
    _map_resolution = resolution_map;
    _discretize_step = _discretize_step_;
}

void VisualizationUtils::visKinoAStarPath(const std::vector<Vec3f> &path) {
    visualization_msgs::MarkerArray line_array;
    visualization_msgs::Marker line;

    line.header.frame_id = "world";
    line.header.stamp = ros::Time::now();
    line.ns = "motion_planner/kino_a_star_path_vis_";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = _map_resolution / 5;

    line.color.r = 0.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    int marker_id = 0;
    line.points.clear();
    geometry_msgs::Point pt;
    line.id = marker_id;
    for (unsigned int index = 0; index < path.size(); index++) {
        Vec3f coord = path[index];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        line.points.push_back(pt);
    }
    line_array.markers.push_back(line);
    _kino_a_star_vis_pub.publish(line_array);
}

void VisualizationUtils::visAStarPath(const std::vector<Vec3f> &path) {
    visualization_msgs::MarkerArray line_array;
    visualization_msgs::Marker line;

    line.header.frame_id = "world";
    line.header.stamp = ros::Time::now();
    line.ns = "motion_planner/a_star_";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = _map_resolution / 5;

    line.color.r = 1.0;
    line.color.g = 0.8;
    line.color.b = 0.3;
    line.color.a = 1.0;

    int marker_id = 0;
    line.points.clear();
    geometry_msgs::Point pt;
    line.id = marker_id;
    for (unsigned int index = 0; index < path.size(); index++) {
        Vec3f coord = path[index];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        line.points.push_back(pt);
    }
    line_array.markers.push_back(line);
    _a_star_vis_pub.publish(line_array);
    
}


void VisualizationUtils::visBreathFirstSearchPath(const std::vector<Vec3f> &path) {
    visualization_msgs::MarkerArray line_array;
    visualization_msgs::Marker line;

    line.header.frame_id = "world";
    line.header.stamp = ros::Time::now();
    line.ns = "motion_planner/breath_first_search";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = _map_resolution / 5;

    line.color.r = 1.0;
    line.color.g = 0.3;
    line.color.b = 1.0;
    line.color.a = 1.0;

    int marker_id = 0;
    line.points.clear();
    geometry_msgs::Point pt;
    line.id = marker_id;
    for (unsigned int index = 0; index < path.size(); index++) {
        Vec3f coord = path[index];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        line.points.push_back(pt);
    }
    line_array.markers.push_back(line);
    _breah_first_search_vis_pub.publish(line_array);
    
}

void VisualizationUtils::visDijkstraPath(const std::vector<Vec3f> &path) {
    visualization_msgs::MarkerArray line_array;
    visualization_msgs::Marker line;

    line.header.frame_id = "world";
    line.header.stamp = ros::Time::now();
    line.ns = "motion_planner/dijkstra_path";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = _map_resolution / 5;

    line.color.r = 0.4;
    line.color.g = 0.4;
    line.color.b = 0.2;
    line.color.a = 1.0;

    int marker_id = 0;
    line.points.clear();
    geometry_msgs::Point pt;
    line.id = marker_id;
    for (unsigned int index = 0; index < path.size(); index++) {
        Vec3f coord = path[index];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);
        line.points.push_back(pt);
    }
    line_array.markers.push_back(line);
    _dijkstra_vis_pub.publish(line_array);
    
}



void VisualizationUtils::PublishPath(const std::vector<Vec3f> &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());
        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    // nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void VisualizationUtils::visKinematicAStarPath(const std::vector<Vec3f> &path, double width,
                                         double length, unsigned int vehicle_interval) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = width;
        vehicle.scale.y = length;
        vehicle.scale.z = 0.01;
        vehicle.color.a = 0.1;

        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;

        vehicle.pose.position.x = path[i].x();
        vehicle.pose.position.y = path[i].y();
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].z());
        vehicle_array.markers.emplace_back(vehicle);
    }

    _kinematic_a_star_pub.publish(vehicle_array);
}

void VisualizationUtils::visRRTPath(std::vector<Vec3f> nodes) {
    visualization_msgs::Marker Points, Line;
    Points.header.frame_id = Line.header.frame_id = "world";
    Points.header.stamp = Line.header.stamp = ros::Time::now();
    Points.ns = Line.ns = "motion_planner/rrt_path";
    Points.action = Line.action = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _map_resolution / 3;
    Points.scale.y = _map_resolution / 3;
    Line.scale.x = _map_resolution / 3;

    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
    Points.color.a = 1.0;
    Line.color.b = 1.0;
    Line.color.a = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Vec3f coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRT_path_vis_pub.publish(Points);
    _RRT_path_vis_pub.publish(Line);
}

void VisualizationUtils::visRRTstarPath(std::vector<Vec3f> nodes) {
    visualization_msgs::Marker Points, Line;
    Points.header.frame_id = Line.header.frame_id = "world";
    Points.header.stamp = Line.header.stamp = ros::Time::now();
    Points.ns = Line.ns = "motion_planner/rrt_star_path";
    Points.action = Line.action = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _map_resolution / 3;
    Points.scale.y = _map_resolution / 3;
    Line.scale.x = _map_resolution / 3;

    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
    Points.color.a = 1.0;
    Line.color.r = 1.0;
    Line.color.a = 1.0;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Vec3f coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line);
}

void VisualizationUtils::visTraLibrary(TrajectoryStatePtr ***TraLibrary) {
    double _map_resolution = 0.2;
    visualization_msgs::MarkerArray LineArray;
    visualization_msgs::Marker Line;

    Line.header.frame_id = "world";
    Line.header.stamp = ros::Time::now();
    Line.ns = "motion_planner/traj_library_";
    Line.action = visualization_msgs::Marker::ADD;
    Line.pose.orientation.w = 1.0;
    Line.type = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x = _map_resolution / 5;

    Line.color.r = 0.0;
    Line.color.g = 0.0;
    Line.color.b = 1.0;
    Line.color.a = 1.0;

    int marker_id = 0;

    for (int i = 0; i <= _discretize_step; i++) {
        for (int j = 0; j <= _discretize_step; j++) {
            for (int k = 0; k <= _discretize_step; k++) {
                if (!TraLibrary[i][j][k]->collision_check) {
                    if (TraLibrary[i][j][k]->optimal_flag) {
                        Line.color.r = 0.0;
                        Line.color.g = 1.0;
                        Line.color.b = 0.0;
                        Line.color.a = 1.0;
                    } else {
                        Line.color.r = 0.0;
                        Line.color.g = 0.0;
                        Line.color.b = 1.0;
                        Line.color.a = 1.0;
                    }
                } else {
                    Line.color.r = 1.0;
                    Line.color.g = 0.0;
                    Line.color.b = 0.0;
                    Line.color.a = 1.0;
                }
                Line.points.clear();
                geometry_msgs::Point pt;
                Line.id = marker_id;
                for (int index = 0; index < int(TraLibrary[i][j][k]->Position.size()); index++) {
                    Vec3f coord = TraLibrary[i][j][k]->Position[index];
                    pt.x = coord(0);
                    pt.y = coord(1);
                    pt.z = coord(2);
                    Line.points.push_back(pt);
                }
                LineArray.markers.push_back(Line);
                _path_vis_pub.publish(LineArray);
                ++marker_id;
            }
        }
    }
}

void VisualizationUtils::visPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_vis) {
    sensor_msgs::PointCloud2 map_vis;
    pcl::toROSMsg(cloud_vis, map_vis);
    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);
}
   