#include <root_finder.h>
#include <minimum_acceleration_obvp.h>
#include <data_type.h>
#include <cubic_spline_interpolation.h>

#include <motion_model.h>
#include <lqr_controller.h>

template <typename T>
std::ostream&  operator<<(std::ostream& os, const std::vector<std::complex<T>>& v) { 
    os << "[";
    for (int i = 0; i < v.size(); ++i) { 
        os <<"( "<< v[i].real() << "," << v[i].imag() << "), "; 
    }
    os << "]\n";
    return os; 
}

template <class T>
std::ostream &operator<<(std::ostream &os, std::vector<T> V)
{
	os << "[ ";
	for (auto v : V)
		os << v << " ";
	return os << "]";
}


int main()
{
    // Related to hw3 
    vec1D waypoints_x = {50, 59, 50, 57, 40, 40};
    vec1D waypoints_y = {25, 12, 10, 2, 4, 14};

    robot_state final_state = {40, 14, 0};
    double ds = 0.1;
    Spline2D ref_traj;
    ref_traj.calculateReferenceTrajectory(waypoints_x, waypoints_y, ds);
    std::string root_dir = "/root/catkin_ws/src/motion_planning/motion_planning_alg/data/";
    ref_traj.saveVector(ref_traj.rx, root_dir+"reference_x.npy");
    ref_traj.saveVector(ref_traj.ry, root_dir+"reference_y.npy");
    ref_traj.saveVector(ref_traj.ryaw, root_dir+"reference_yaw.npy");
    ref_traj.saveVector(ref_traj.rk, root_dir+"reference_curvature.npy");
    ref_traj.saveVector(waypoints_x, root_dir+"waypoints_x.npy");
    ref_traj.saveVector(waypoints_y, root_dir+"waypoints_y.npy");

    double kp = 1.0;
    int T = 800;
    double L = 1.5;
    double delta_t = 0.1;
    double target_velocity = 1.3;
    robot_state start_p = {40, 15, 0};
    LQRControl lqr_controller(kp);
    HagenRobot robot;
    robot.initRobot(start_p, target_velocity, L, delta_t);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(HagenRobot::N_X, HagenRobot::N_X)*3;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(HagenRobot::N_U, HagenRobot::N_U)*2;
    Eigen::MatrixXd A((int)HagenRobot::N_X, (int)HagenRobot::N_X);
    Eigen::MatrixXd B((int)HagenRobot::N_X, (int)HagenRobot::N_U);

    double delta_f = 0;
    vec1D traversed_path_x, traversed_path_y;

    for(int i=0; i<T; i++){
        
        robot_state current_state = robot.getCurrentState();
        std::cout<< current_state << std::endl;
        double e, k, ref_yaw;
        int s;
        ref_traj.calculateTrackingError(current_state.x, current_state.y, e, k, ref_yaw, s);
        double ref_delta_f = atan2(L*k, 1.0);
        robot.getLinearModel(ref_delta_f, ref_yaw, A, B);
        robot_state reference_state = ref_traj.getState(s);
        auto control  = lqr_controller.lqrControl(current_state, reference_state, A, B, Q, R);
        delta_f = control(1);
        delta_f += ref_delta_f;
        double a = lqr_controller.pControl(target_velocity, robot.v);
        robot.step(delta_f, a);
        traversed_path_x.push_back(robot.getCurrentState().x);
        traversed_path_y.push_back(robot.getCurrentState().y);

        if((current_state-final_state).norm()<0.01){
            std::cout<< "Robot reached to the target pose" << std::endl;
            break;
        }
    }

    ref_traj.saveVector(traversed_path_x, root_dir+"traversed_path_x.npy");
    ref_traj.saveVector(traversed_path_y, root_dir+"traversed_path_y.npy");

    return 0;
}