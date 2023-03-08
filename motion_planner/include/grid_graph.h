#ifndef GRID_GRAPH
#define GRID_GRAPH

#include <iostream>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <cmath>
#include <random>
#include <type_traits>
#include "data_type.h"
#include "robot_state.h"
#include "types.hpp"

namespace hagen_planner{ 

template<typename State>
class GridGraph {
  public:
   
    void InitGridMap(const Eigen::Vector3d &xyz_coordinate_lower, const Eigen::Vector3d &xyz_coordinate_upper,
                        const Eigen::Vector3i &grid_size, const double &grid_resolution);
    
    void InitGridMap(double map_grid_resolution, double map_upper_x, double map_upper_y, kino_planner::SearchInfo& params);
    Vec3f gridIndex2coord(const Vec3i &index);
    Vec3i coord2gridIndex(const Vec3f &pt);
    Vec3f coordRounding(const Vec3f &coord);
    void getNeighbors(typename State::Ptr currentPtr, std::vector< typename State::Ptr> &neighbors);
    void getNeighbors(TrajectoryStatePtr ***trajectory_state_ptr, std::vector< typename State::Ptr> &neighbors,
                                                                          std::vector<TrajectoryStatePtr> &neighbors_traj_state, const int discretize_step);
    
    Eigen::Vector3d checkPointRange(const Eigen::Vector3d &point);
    void setObs(const Eigen::Vector3d &obstacle_coord);
    bool isObsFree(const double coord_x, const double coord_y, const double coord_z);
    bool isObsFree(const Vec3i idx);
    bool collisionFree(const Eigen::Vector3d &near_point, const Eigen::Vector3d &new_point);
    bool isObstacle(const Eigen::Vector3d &point_coord);

    bool LineCheck(double x0, double y0, double x1, double y1);
    bool CheckCollision(const double &x, const double &y, const double &theta);
    bool HasObstacle(const int grid_index_x, const int grid_index_y) const;
    bool HasObstacle(const Vec2i &grid_index) const;
    void SetObstacle(unsigned int x, unsigned int y);
    void SetObstacle(const double pt_x, const double pt_y);
    bool BeyondBoundary(const Vec2d &pt) const;
    void GetNeighborNodes(const typename State::Ptr &curr_node_ptr, std::vector<typename State::Ptr> &neighbor_nodes);
    Vec2i Coordinate2MapGridIndex(const Vec2d &pt) const ;
    Vec3i State2Index(const Vec3d &state) const;
    Vec3i State2IndexInit(const Vec3d &state) const;
    Vec2d MapGridIndex2Coordinate(const Vec2i &grid_index) const ;
    __attribute__((unused)) Vec2d  CoordinateRounding(const Vec2d &pt) const;
    __attribute__((unused)) int GetVisitedNodesNumber() const { return visited_node_number_; }
    void SetVehicleShape(double length, double width, double rear_axle_dist);
    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta);
    double Mod2Pi(const double &x);
    void ReleaseMemory();
    vec_Vec4f GetSearchedTree();

    Eigen::Vector3d Sample();

    void reset();

    std::vector<std::vector<std::vector<typename State::Ptr>>> GridNodeMap_;
    int GLX_SIZE_, GLY_SIZE_, GLZ_SIZE_, GLXY_SIZE_;
    int GLXYZ_SIZE_, GLYZ_SIZE_;
    double resolution_, inv_resolution_;
    std::vector<uint8_t> data;

    Eigen::Vector3d xyz_coord_lower_;
    Eigen::Vector3d xyz_coord_upper_;
    Eigen::Vector3i grid_size_;

    std::vector<uint8_t> map_data_;
    double STATE_GRID_RESOLUTION_{}, MAP_GRID_RESOLUTION_{};
    double ANGULAR_RESOLUTION_{};
    int STATE_GRID_SIZE_X_{}, STATE_GRID_SIZE_Y_{}, STATE_GRID_SIZE_PHI_{};
    int MAP_GRID_SIZE_X_{}, MAP_GRID_SIZE_Y_{};
    double map_x_lower_{}, map_x_upper_{}, map_y_lower_{}, map_y_upper_{};
    typename State::Ptr ***state_node_map_ = nullptr;
    // std::vector<std::vector<std::vector<typename State::Ptr>>> state_node_map_;
    int num_check_collision = 0;
    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;
    int steering_discrete_num_;
    double steering_radian_step_size_;
    int segment_length_discrete_num_;
    double move_step_size_;
    double segment_length_;
    double wheel_base_; 
    double steering_radian_; //radian
    int visited_node_number_ = 0;

};
};
typedef hagen_planner::GridGraph<RobotNode> GridGraph3D;
typedef hagen_planner::GridGraph<RRTNode> GridRRTGraph3D;
#endif 