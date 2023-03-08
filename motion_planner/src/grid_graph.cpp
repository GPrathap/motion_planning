#include "grid_graph.h"
#include <cmath>
#include <unordered_set>

namespace hagen_planner
{
    using namespace std;


    template<typename State>
    void GridGraph<State>::InitGridMap(const Eigen::Vector3d &xyz_coordinate_lower, const Eigen::Vector3d &xyz_coordinate_upper,
                        const Eigen::Vector3i &grid_size, const double &grid_resolution) {

            xyz_coord_lower_ = xyz_coordinate_lower;
            xyz_coord_upper_ = xyz_coordinate_upper;
            grid_size_ = grid_size;
           
            GLX_SIZE_ = grid_size_[0];
            GLY_SIZE_ = grid_size_[1];
            GLZ_SIZE_ = grid_size_[2];
            GLXY_SIZE_ = GLX_SIZE_ * GLY_SIZE_;
            GLYZ_SIZE_ = GLY_SIZE_ * GLZ_SIZE_;
            GLXYZ_SIZE_ = GLX_SIZE_ * GLYZ_SIZE_;

            resolution_ = grid_resolution;
            inv_resolution_ = 1.0 / grid_resolution;
            data.assign(GLXYZ_SIZE_, 0);
            GridNodeMap_.resize(GLX_SIZE_, std::vector<std::vector< typename State::Ptr>>(GLY_SIZE_, std::vector< typename State::Ptr>(GLZ_SIZE_, nullptr)));

            for (int i = 0; i < GLX_SIZE_; ++i)
            {
                for (int j = 0; j < GLY_SIZE_; ++j)
                {
                    for (int k = 0; k < GLZ_SIZE_; ++k)
                    {
                        Vec3i tempIdx(i, j, k);
                        Vec3f pos = gridIndex2coord(tempIdx);
                        if constexpr (std::is_same_v<State, RobotNode>)
                        {
                            GridNodeMap_[i][j][k] = new State(tempIdx, pos);
                        }
                    }
                }
            }
        }

    template<typename State>
    void GridGraph<State>::InitGridMap(double map_grid_resolution, double map_upper_x, double map_upper_y, kino_planner::SearchInfo& params) {
        
        wheel_base_ = params.wheel_base;
        steering_discrete_num_ = params.steering_angle_discrete_num;
        steering_radian_ = params.steering_angle * M_PI / 180.0; // angle to radian
        steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
        move_step_size_ = params.segment_length / params.segment_length_discrete_num;
        
        segment_length_discrete_num_ = static_cast<int>(params.segment_length_discrete_num);
        // CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        // << "The segment length must be divisible by the step size. segment_length: "
        // << segment_length_ << " | step_size: " << move_step_size_;
        STATE_GRID_SIZE_PHI_ = params.grid_size_phi;
        ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;

        auto map_resolution = 1.0;
        map_x_lower_ = 0;
        map_x_upper_ = 2*xyz_coord_upper_(0)/map_grid_resolution;
        map_y_lower_ = 0;
        map_y_upper_ = 2*xyz_coord_upper_(1)/map_grid_resolution;

        std::cerr << map_x_lower_ << " " << map_x_upper_ << " " << map_y_lower_ << " " <<  map_y_upper_ << std::endl;
        STATE_GRID_RESOLUTION_ = map_resolution;
        MAP_GRID_RESOLUTION_ = map_resolution;
        
        STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
        STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

        MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
        MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

        map_data_.clear();
        map_data_.assign(MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_, 0);
        state_node_map_ = new typename State::Ptr **[STATE_GRID_SIZE_X_];
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            state_node_map_[i] = new typename State::Ptr *[STATE_GRID_SIZE_Y_];
            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                state_node_map_[i][j] = new typename State::Ptr[STATE_GRID_SIZE_PHI_];
                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    state_node_map_[i][j][k] = nullptr;
                }
            }
        }
        SetVehicleShape(params.vehicle_length, params.vehicle_width, params.track);
    }

    template<typename State>
    vec_Vec4f GridGraph<State>::GetSearchedTree() {
            vec_Vec4f tree;
            if constexpr (std::is_same_v<State, RobotNode>){
                Vec4d point_pair;
                visited_node_number_ = 0;
                for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
                    for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                        for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                            if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {
                                continue;
                            }

                            const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                            for (unsigned int l = 0; l < number_states; ++l) {
                                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                                point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);

                                tree.emplace_back(point_pair);
                            }

                            point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                            point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->robot_state_.head(2);
                            tree.emplace_back(point_pair);
                            visited_node_number_++;
                        }
                    }
                }
            }
            return tree;
    }


    template<typename State>
    bool GridGraph<State>::LineCheck(double x0, double y0, double x1, double y1) {
        bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

        if (steep) {
            std::swap(x0, y0);
            std::swap(y1, x1);
        }

        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        auto delta_x = x1 - x0;
        auto delta_y = std::abs(y1 - y0);
        auto delta_error = delta_y / delta_x;
        decltype(delta_x) error = 0;
        decltype(delta_x) y_step;
        auto yk = y0;

        if (y0 < y1) {
            y_step = 1;
        } else {
            y_step = -1;
        }

        auto N = static_cast<unsigned int>(x1 - x0);
        for (unsigned int i = 0; i < N; ++i) {
            if (steep) {
                if (HasObstacle(Vec2i(yk, x0 + i * 1.0))|| BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_, (x0 + i) * MAP_GRID_RESOLUTION_))) {
                    return false;
                }
            } else {
                if (HasObstacle(Vec2i(x0 + i * 1.0, yk))|| BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_, yk * MAP_GRID_RESOLUTION_))) {
                    return false;
                }
            }
            error += delta_error;
            if (error >= 0.5) {
                yk += y_step;
                error = error - 1.0;
            }
        }

        return true;
    }

    template<typename State>
    bool GridGraph<State>::BeyondBoundary(const Vec2d &pt) const {
        return pt.x() < map_x_lower_ || pt.x() >= map_x_upper_ || pt.y() < map_y_lower_ || pt.y() >= map_y_upper_;
    }


    template<typename State>
    bool GridGraph<State>::CheckCollision(const double &x, const double &y, const double &theta) {
        Mat2d R;
        R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
        MatXd transformed_vehicle_shape;
        transformed_vehicle_shape.resize(8, 1);
        for (unsigned int i = 0; i < 4u; ++i) {
            transformed_vehicle_shape.block<2, 1>(i * 2, 0) = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
        }

        Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(0, 0));
        Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(2, 0));
        Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(4, 0));
        Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(transformed_vehicle_shape.block<2, 1>(6, 0));

        double y1, y0, x1, x0;
        // pt1 -> pt0
        x0 = static_cast<double>(transformed_pt_index_0.x());
        y0 = static_cast<double>(transformed_pt_index_0.y());
        x1 = static_cast<double>(transformed_pt_index_1.x());
        y1 = static_cast<double>(transformed_pt_index_1.y());

        if (!LineCheck(x1, y1, x0, y0)) {
            return false;
        }
        // pt2 -> pt1
        x0 = static_cast<double>(transformed_pt_index_1.x());
        y0 = static_cast<double>(transformed_pt_index_1.y());
        x1 = static_cast<double>(transformed_pt_index_2.x());
        y1 = static_cast<double>(transformed_pt_index_2.y());

        if (!LineCheck(x1, y1, x0, y0)) {
            return false;
        }

        // pt3 -> pt2
        x0 = static_cast<double>(transformed_pt_index_2.x());
        y0 = static_cast<double>(transformed_pt_index_2.y());
        x1 = static_cast<double>(transformed_pt_index_3.x());
        y1 = static_cast<double>(transformed_pt_index_3.y());

        if (!LineCheck(x1, y1, x0, y0)) {
            return false;
        }

        // pt0 -> pt3
        x0 = static_cast<double>(transformed_pt_index_0.x());
        y0 = static_cast<double>(transformed_pt_index_0.y());
        x1 = static_cast<double>(transformed_pt_index_3.x());
        y1 = static_cast<double>(transformed_pt_index_3.y());

        if (!LineCheck(x0, y0, x1, y1)) {
            return false;
        }
        num_check_collision++;
        return true;
    }

    template<typename State>
    bool GridGraph<State>::HasObstacle(const int grid_index_x, const int grid_index_y) const {
        return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
                && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
                && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
    }

    template<typename State>
    bool GridGraph<State>::HasObstacle(const Vec2i &grid_index) const {
        int grid_index_x = grid_index[0];
        int grid_index_y = grid_index[1];

        return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
                && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
                && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
    }

    template<typename State>
    void GridGraph<State>::SetObstacle(unsigned int x, unsigned int y) {
        if (x < 0u || x > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
            || y < 0u || y > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {
            return;
        }
        map_data_[x + y * MAP_GRID_SIZE_X_] = 1;
    }

    template<typename State>
    void GridGraph<State>::SetObstacle(const double pt_x, const double pt_y) {
        if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
            pt_y < map_y_lower_ || pt_y > map_y_upper_) {
            return;
        }

        int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
        int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

        map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
    }

    /*!
     * Set vehicle shape
     * Consider the shape of the vehicle as a rectangle.
     * @param length vehicle length (a to c)
     * @param width vehicle width (a to d)
     * @param rear_axle_dist Length from rear axle to rear (a to b)
     *
     *         b
     *  a  ---------------- c
     *    |    |          |    Front
     *    |    |          |
     *  d  ----------------
     */
    template<typename State>
    void GridGraph<State>::SetVehicleShape(double length, double width, double rear_axle_dist) {
        vehicle_shape_.resize(8);
        vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
        vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
        vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
        vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

        const double step_size = move_step_size_;
        const auto N_length = static_cast<unsigned int>(length / step_size);
        const auto N_width = static_cast<unsigned int> (width / step_size);
        vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

        const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                        - vehicle_shape_.block<2, 1>(0, 0)).normalized();
        for (unsigned int i = 0; i < N_length; ++i) {
            vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                    = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
            vehicle_shape_discrete_.block<2, 1>(0, i)
                    = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
        }

        const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                        - vehicle_shape_.block<2, 1>(2, 0)).normalized();
        for (unsigned int i = 0; i < N_width; ++i) {
            vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                    = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
            vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                    = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
        }
    }

    template<typename State> __attribute__((unused)) Vec2d  GridGraph<State>::CoordinateRounding(const Vec2d &pt) const {
        return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
    }



    template<typename State>
    void  GridGraph<State>::GetNeighborNodes(const typename State::Ptr &curr_node_ptr, std::vector<typename State::Ptr> &neighbor_nodes) {
        neighbor_nodes.clear();
        if constexpr (std::is_same_v<State, RobotNode>){
            for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
                vec_Vec3f intermediate_state;
                bool has_obstacle = false;

                double x = curr_node_ptr->robot_state_.x();
                double y = curr_node_ptr->robot_state_.y();
                double theta = curr_node_ptr->robot_state_.z();

                const double phi = i * steering_radian_step_size_;
                // forward
                for (int j = 1; j <= segment_length_discrete_num_; j++) {
                    DynamicModel(move_step_size_, phi, x, y, theta);
                    intermediate_state.emplace_back(Vec3d(x, y, theta));
                    if (!CheckCollision(x, y, theta)) {
                        has_obstacle = true;
                        break;
                    }
                }
                Vec3i grid_index = State2Index(intermediate_state.back());
                if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
                    auto neighbor_forward_node_ptr = new State(grid_index);
                    neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
                    neighbor_forward_node_ptr->robot_state_ = intermediate_state.back();
                    neighbor_forward_node_ptr->steering_grade_ = i;
                    neighbor_forward_node_ptr->direction_ = State::FORWARD;
                    neighbor_nodes.push_back(neighbor_forward_node_ptr);
                }
                // backward
                has_obstacle = false;
                intermediate_state.clear();
                x = curr_node_ptr->robot_state_.x();
                y = curr_node_ptr->robot_state_.y();
                theta = curr_node_ptr->robot_state_.z();
                for (int j = 1; j <= segment_length_discrete_num_; j++) {
                    DynamicModel(-move_step_size_, phi, x, y, theta);
                    intermediate_state.emplace_back(Vec3d(x, y, theta));
                    if (!CheckCollision(x, y, theta)) {
                        has_obstacle = true;
                        break;
                    }
                }
                if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
                    grid_index = State2Index(intermediate_state.back());
                    auto neighbor_backward_node_ptr = new State(grid_index);
                    neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
                    neighbor_backward_node_ptr->robot_state_ = intermediate_state.back();
                    neighbor_backward_node_ptr->steering_grade_ = i;
                    neighbor_backward_node_ptr->direction_ = State::BACKWARD;
                    neighbor_nodes.push_back(neighbor_backward_node_ptr);
                }
            }
        }
    }

    template<typename State>
    void GridGraph<State>::DynamicModel(const double &step_size, const double &phi,
                                double &x, double &y, double &theta)  {
        x = x + step_size * std::cos(theta);
        y = y + step_size * std::sin(theta);
        theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
    }


    template<typename State>
    double GridGraph<State>::Mod2Pi(const double &x) {
        double v = fmod(x, 2 * M_PI);
    
        if (v < -M_PI) {
            v += 2.0 * M_PI;
        } else if (v > M_PI) {
            v -= 2.0 * M_PI;
        }

        return v;
    }



    template<typename State>
    Eigen::Vector3d GridGraph<State>::Sample() {
        std::random_device rd;
        std::default_random_engine eng(rd());

        std::uniform_real_distribution<> rand_pt_x =
                std::uniform_real_distribution<>(xyz_coord_lower_.x(), xyz_coord_upper_.x());
        std::uniform_real_distribution<> rand_pt_y =
                std::uniform_real_distribution<>(xyz_coord_lower_.y(), xyz_coord_upper_.y());
        std::uniform_real_distribution<> rand_pt_z =
                std::uniform_real_distribution<>(xyz_coord_lower_.z(), xyz_coord_upper_.z());

        Eigen::Vector3d rand_pt;
        rand_pt << rand_pt_x(eng), rand_pt_y(eng), rand_pt_z(eng);

        return rand_pt;
    }

    template <typename State>
    void GridGraph<State>::setObs(const Eigen::Vector3d &obstacle_coord)
    {
        if (obstacle_coord[0] < xyz_coord_lower_(0) || obstacle_coord[1] < xyz_coord_lower_(1) || obstacle_coord[2] < xyz_coord_lower_(2) ||
            obstacle_coord[0] >= xyz_coord_upper_(0) || obstacle_coord[1] >= xyz_coord_upper_(1) || obstacle_coord[2] >= xyz_coord_upper_(2))
            return;

        int idx_x = static_cast<int>((obstacle_coord[0] - xyz_coord_lower_(0)) * inv_resolution_);
        int idx_y = static_cast<int>((obstacle_coord[1] - xyz_coord_lower_(1)) * inv_resolution_);
        int idx_z = static_cast<int>((obstacle_coord[2] - xyz_coord_lower_(2)) * inv_resolution_);

        data[idx_x * GLYZ_SIZE_ + idx_y * GLZ_SIZE_ + idx_z] = 1;
    }

    template<typename State>
    bool GridGraph<State>::isObstacle(const Eigen::Vector3d &point_coord) {
        const Eigen::Vector3i point_index = coord2gridIndex(point_coord);

        int grid_index = GLXY_SIZE_ * point_index.z() + GLX_SIZE_ * point_index.y() + point_index.x();

        return data[grid_index];
    }

    template<typename State>
    bool GridGraph<State>::collisionFree(const Eigen::Vector3d &near_point, const Eigen::Vector3d &new_point) {
        double step_size = resolution_ / 5.0;
        Eigen::Vector3d direction = (new_point-near_point).normalized();
        Eigen::Vector3d delta = direction * step_size;
        Eigen::Vector3d temp_point = near_point;
        double dist_thred = (new_point - near_point).norm();
        while (true){
            temp_point = temp_point + delta;

            if ((temp_point - near_point).norm() >= dist_thred){
                return true;
            }

            if (isObstacle(temp_point)){
                return false;
            }
        }
    }



    template <typename State>
    bool GridGraph<State>::isObsFree(const double coord_x, const double coord_y, const double coord_z)
    {
        Vec3f pt;
        Vec3i idx;

        pt(0) = coord_x;
        pt(1) = coord_y;
        pt(2) = coord_z;
        idx = coord2gridIndex(pt);

        int idx_x = idx(0);
        int idx_y = idx(1);
        int idx_z = idx(2);

        return (idx_x >= 0 && idx_x < GLX_SIZE_ && idx_y >= 0 && idx_y < GLY_SIZE_ && idx_z >= 0 && idx_z < GLZ_SIZE_ &&
                (data[idx_x * GLYZ_SIZE_ + idx_y * GLZ_SIZE_ + idx_z] < 1));
    }

    template <typename State>
    bool GridGraph<State>::isObsFree(const Vec3i idx)
    {

        int idx_x = idx(0);
        int idx_y = idx(1);
        int idx_z = idx(2);

        return (idx_x >= 0 && idx_x < GLX_SIZE_ && idx_y >= 0 && idx_y < GLY_SIZE_ && idx_z >= 0 && idx_z < GLZ_SIZE_ &&
                (data[idx_x * GLYZ_SIZE_ + idx_y * GLZ_SIZE_ + idx_z] < 1));
    }




    template<typename State>
    Eigen::Vector3d GridGraph<State>::checkPointRange(const Eigen::Vector3d &point) {
        Eigen::Vector3d corrected_point;
        corrected_point.x() = std::min(std::max(xyz_coord_lower_.x(), point.x()), xyz_coord_upper_.x());
        corrected_point.y() = std::min(std::max(xyz_coord_lower_.y(), point.y()), xyz_coord_upper_.y());
        corrected_point.z() = std::min(std::max(xyz_coord_lower_.z(), point.z()), xyz_coord_upper_.z());

        return corrected_point;
    }

    template <typename State>
    Vec3f GridGraph<State>::gridIndex2coord(const Vec3i &index)
    {
        Vec3f pt;

        pt(0) = ((double)index(0) + 0.5) * resolution_ + xyz_coord_lower_(0);
        pt(1) = ((double)index(1) + 0.5) * resolution_ + xyz_coord_lower_(1);
        pt(2) = ((double)index(2) + 0.5) * resolution_ + xyz_coord_lower_(2);

        return pt;
    }

    template <typename State>
    Vec3i GridGraph<State>::coord2gridIndex(const Vec3f &pt)
    {
        Vec3i idx;
        idx << min(max(int((pt(0) - xyz_coord_lower_(0)) * inv_resolution_), 0), GLX_SIZE_ - 1),
            min(max(int((pt(1) - xyz_coord_lower_(1)) * inv_resolution_), 0), GLY_SIZE_ - 1),
            min(max(int((pt(2) - xyz_coord_lower_(2)) * inv_resolution_), 0), GLZ_SIZE_ - 1);

        return idx;
    }


    template<typename State>
    Vec3i  GridGraph<State>::State2Index(const Vec3d &state) const {
        Vec3i index;
        index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
        index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
        index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);
        return index;
    }

    template<typename State>
    Vec3i  GridGraph<State>::State2IndexInit(const Vec3d &state) const {
        Vec3i index;
        index[0] = min(max(int((state(0) - xyz_coord_lower_(0)) * inv_resolution_), 0), GLX_SIZE_ - 1);
        index[1] = min(max(int((state(1) - xyz_coord_lower_(1)) * inv_resolution_), 0), GLY_SIZE_ - 1);
        index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);
        return index;
    }


    template<typename State>
    Vec2d  GridGraph<State>::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
        Vec2d pt;
        pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
        pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;
        return pt;
    }




    template<typename State>
    Vec2i  GridGraph<State>::Coordinate2MapGridIndex(const Vec2d &pt) const {
        Vec2i grid_index;

        grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
        grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
        return grid_index;
    }


    template <typename State>
    Vec3f GridGraph<State>::coordRounding(const Vec3f &coord)
    {
        return gridIndex2coord(coord2gridIndex(coord));
    }

    template <typename State>
    void GridGraph<State>::getNeighbors( typename State::Ptr currentPtr, std::vector< typename State::Ptr> &neighbors)
    {
        if constexpr (std::is_same_v<State, RobotNode>){
            neighbors.clear();
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dz = -1; dz <= 1; ++dz)
                    {
                        Vec3i idx = currentPtr->robot_grid_index_ + Vec3i(dx, dy, dz);
                        if (idx == currentPtr->robot_grid_index_)
                            continue;
                        if (idx.x() < 0 || idx.x() >= GLX_SIZE_ || idx.y() < 0 || idx.y() >= GLY_SIZE_ || idx.z() < 0 || idx.z() >= GLZ_SIZE_)
                            continue;

                        if (!isObsFree(idx))
                        {
                            continue;
                        }
                        typename State::Ptr current_grid_node_ptr = GridNodeMap_[idx.x()][idx.y()][idx.z()];
                        if (current_grid_node_ptr->id_ == State::ROBOT_NODE_STATUS::WAS_THERE)
                        {
                            continue;
                        }
                        neighbors.emplace_back(current_grid_node_ptr);
                    }
                }
            }
        }
    }


    template<typename State>
    void GridGraph<State>::getNeighbors(TrajectoryStatePtr ***trajectory_state_ptr, std::vector< typename State::Ptr> &neighbors,
                            vector<TrajectoryStatePtr> &neighbors_traj_state, const int discretize_step_) {
        neighbors.clear();
        neighbors_traj_state.clear();
        std::unordered_map<int, std::pair< typename State::Ptr, TrajectoryStatePtr>> candidates_grid_node_traj;
        if constexpr (std::is_same_v<State, RobotNode>){
                for (int i = 0; i < discretize_step_; ++i) {
                for (int j = 0; j < discretize_step_; ++j) {
                    for (int k = 0; k < discretize_step_; ++k) {
                        auto current_trajectory_state_ptr = trajectory_state_ptr[i][j][k];

                        if (current_trajectory_state_ptr->collision_check) {
                            delete current_trajectory_state_ptr;
                            continue;
                        }

                        Vec3f coord_end = current_trajectory_state_ptr->Position.back();
                        Vec3i index_end = coord2gridIndex(coord_end);

                        const int index = k * i * j + j * i + i;
                        if (candidates_grid_node_traj.count(index) == 0) {
                            typename State::Ptr current_grid_node_ptr;
                            current_grid_node_ptr = GridNodeMap_[index_end.x()][index_end.y()][index_end.z()];
                            current_grid_node_ptr->robot_state_ = coord_end;
                            candidates_grid_node_traj[index] = std::make_pair(current_grid_node_ptr, current_trajectory_state_ptr);
                        } else {
                            if (current_trajectory_state_ptr->Trajctory_Cost <
                                candidates_grid_node_traj[index].second->Trajctory_Cost) {
                                typename State::Ptr current_grid_node_ptr;
                                current_grid_node_ptr = GridNodeMap_[index_end.x()][index_end.y()][index_end.z()];
                                current_grid_node_ptr->robot_state_ = coord_end;
                                candidates_grid_node_traj[index] = std::make_pair(current_grid_node_ptr, current_trajectory_state_ptr);
                            }
                        }
                    }
                }
            }

            auto iter = candidates_grid_node_traj.begin();
            for (; iter != candidates_grid_node_traj.end(); ++iter) {
                neighbors.emplace_back(iter->second.first);
                neighbors_traj_state.emplace_back(iter->second.second);
            }
        }
    }

    template <typename State>
    void GridGraph<State>::reset()
    {
        for (int i = 0; i < GLX_SIZE_; i++)
            for (int j = 0; j < GLY_SIZE_; j++)
                for (int k = 0; k < GLZ_SIZE_; k++)
                {
                    if constexpr (std::is_same_v<State, RobotNode>){
                        if (GridNodeMap_[i][j][k] != nullptr)
                        {
                            GridNodeMap_[i][j][k]->id_ = State::ROBOT_NODE_STATUS::WOULD_LIKE;
                            GridNodeMap_[i][j][k]->g_score_ = std::numeric_limits<double>::max();
                            GridNodeMap_[i][j][k]->f_score_ = std::numeric_limits<double>::max();
                            GridNodeMap_[i][j][k]->parent_node_ = nullptr;
                            GridNodeMap_[i][j][k]->trajectory_ = nullptr;
                        }
                    }
                }
    }

    template<typename State>
    void GridGraph<State>::ReleaseMemory() {
    }

    
};

template class hagen_planner::GridGraph<RobotNode>;
template class hagen_planner::GridGraph<RRTNode>;
