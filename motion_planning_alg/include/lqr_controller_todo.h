#ifndef LQR_CONTROLLER
#define LQR_CONTROLLER

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <vector>
#include <complex>
#include <cfenv>
#include "motion_model.h"

template<typename robot_motion_model>
class LQRController {

    private:
        const double kp;
        const double allowed_error;
        int N = 100;
        const int n_x = robot_motion_model::N_X;
        const int n_u = robot_motion_model::N_U;
        Eigen::MatrixXd Q_int;

    public:

        LQRController(const double kp_=1, const double allowed_error_=1e-2):kp(kp_)
                                                , allowed_error(allowed_error_){

                Q_int =  Eigen::MatrixXd::Identity(n_x, n_x);
        }

        Eigen::MatrixXd calculateDARE(Eigen::MatrixXd& A, Eigen::MatrixXd& B
                        , Eigen::MatrixXd& Q, Eigen::MatrixXd& R){

            Eigen::MatrixXd P = Q_int; 
            Eigen::MatrixXd P_; 
            for(int i=0; i<N; i++){
                // TODO calculate DARE, see the lectures
                // P_ = 
                double max_val = (P_-P).cwiseAbs().maxCoeff();
                if(max_val < allowed_error){
                    break;
                }
                P = P_;
            }
            return P_;
        }

        double lqrControl(robot_state state, robot_state ref_state
            , Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& Q, Eigen::MatrixXd& R){
                Eigen::VectorXd x_diff = state - ref_state;
                Eigen::MatrixXd P = calculateDARE(A, B, Q, R);
                Eigen::MatrixXd K;
                // TODO estimate the K matrix, see the lectures 
                auto u = K*x_diff;
                return u(1); 
        }

        double pControl(const double desired, const double current){
            return kp*(desired-current);
        }

        ~LQRController() = default;
};

typedef LQRController<HagenRobot> LQRControl;
#endif //LQR_CONTROLLER
