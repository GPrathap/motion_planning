#ifndef MOTION_MODEL
#define MOTION_MODEL

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <vector>
#include <complex>
#include <cfenv>

struct RobotState {
    double x;
    double y;
    double theta;
    int size = 3; 

    Eigen::VectorXd  operator-(const RobotState& state){
        Eigen::VectorXd diff(size);
        diff << this->x - state.x, this->y - state.y, this->theta - state.theta;
        return diff;
    }

    friend std::ostream &operator<<(std::ostream &os, RobotState V)
    {
        os << " robot state: " << V.x << "," << V.y << ", " << V.theta << " " << std::endl;
    }

};

typedef RobotState robot_state;

template<typename State> 
class MotionModel {
    public:
        MotionModel() = default;
        ~MotionModel() = default;

        void initRobot(const State state, const double v_
                    , const double L_=2.4, const double delta_t_=0.01){
            current_state = state;
            L = L_;
            v = v_;
            delta_t = delta_t_;
        }

        void step(const double delta_f, const double acc){
            current_state.x += v*cos(current_state.theta)*delta_t;
            current_state.y += v*sin(current_state.theta)*delta_t;
            current_state.theta += v*(tan(delta_f)/L)*delta_t;
            current_state.theta = std::fmod(current_state.theta, M_PI*2);
            v += acc*delta_t;
        }

        State getCurrentState(){
            return current_state;
        }

        void getLinearModel(double ref_delta_f, double ref_yaw
                        , Eigen::MatrixXd& A, Eigen::MatrixXd& B){

            A.row(0) << 1.0, 0.0, -v*delta_t*sin(ref_yaw);
            A.row(1) << 0.0, 1.0, v*delta_t*cos(ref_yaw);
            A.row(2) << 0.0, 0.0, 1.0;

            B.row(0) << delta_t*cos(ref_yaw), 0;
            B.row(1) << delta_t*sin(ref_yaw), 0;
            B.row(2) << delta_t*tan(ref_delta_f)/L
                    , v*delta_t*(1.0/(L*cos(ref_delta_f)*cos(ref_delta_f)));
            
        }

        static const int N_X = 3;
        static const int N_U = 2;
        double v;
       
    private:
        State current_state;
        double L;
        double delta_t;
};

typedef MotionModel<RobotState> HagenRobot;

#endif //MOTION_MODEL
