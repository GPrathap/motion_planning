#ifndef GRID_BASED_A_STAR_PLANNER
#define GRID_BASED_A_STAR_PLANNER

#include "robot_state.h"
#include "grid_graph.h"
#include <Eigen/Dense>

using namespace Eigen;

template<typename Graph, typename State>
class AStar {
    public:
        AStar() = default;
        ~AStar() = default;
        void searchPath(const Vec3f &start_pt, const Vec3f &end_pt
                                , std::function<huristics_cost_t( typename State::Ptr a,  typename State::Ptr b)> calculate_huristics);
        std::vector<Vec3f> getPath();
        void setGraph(std::shared_ptr<Graph> graph);
        
        void dijkstraSearchPath(const Vec3f &start_pt, const Vec3f &end_pt,
                     std::function<huristics_cost_t( typename State::Ptr a,  typename State::Ptr b)> calculate_huristics);

        void breadthFirstSearch(const Vec3f &start_pt, const Vec3f &end_pt,
                     std::function<huristics_cost_t( typename State::Ptr a,  typename State::Ptr b)> calculate_huristics);

    private:
         typename State::Ptr terminate_ptr_;
        std::multimap<double,  typename State::Ptr> open_set_;
        std::shared_ptr<Graph> graph_;
};

typedef AStar<GridGraph3D, RobotNode> AStar3D;
#endif //GRID_BASED_A_STAR_PLANNER
