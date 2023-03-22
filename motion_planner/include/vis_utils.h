#ifndef ROBOT_VIS_UTILS
#define ROBOT_VIS_UTILS

#include <iostream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <iomanip>
#include "data_type.h"
#include "State.h"


#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>


class VisualizationUtils {
    public:
        VisualizationUtils() = default;
        ~VisualizationUtils() = default;
        void init(ros::NodeHandle& nh, double resolution_map, double _discretize_step_);
        void visKinoAStarPath(const std::vector<Vec3f> &path);
        void visAStarPath(const std::vector<Vec3f> &path);
        void visDijkstraPath(const std::vector<Vec3f> &path);
        void visBreathFirstSearchPath(const std::vector<Vec3f> &path);
        void visTraLibrary(MotionStateMapPtr motionPrimitives);
        void visPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_vis);
        void visRRTPath(std::vector<Vec3f> nodes);
        void visRRTstarPath(std::vector<Vec3f> nodes);
        void visKinematicAStarPath(const std::vector<Vec3f> &path, double width,
                                         double length, unsigned int vehicle_interval = 5u);
        void PublishPath(const std::vector<Vec3f> &path);
    private:
        ros::Publisher _grid_map_vis_pub, _path_vis_pub, _kino_a_star_vis_pub, _a_star_vis_pub, _kinematic_a_star_pub;
        ros::Publisher _RRTstar_path_vis_pub, _RRT_path_vis_pub, path_pub_, _dijkstra_vis_pub, _breah_first_search_vis_pub;
        double _map_resolution;
        double _discretize_step;
};
#endif //ROBOT_VIS_UTILS