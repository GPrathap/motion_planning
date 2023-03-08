#ifndef MIN_ACC_OBVP
#define MIN_ACC_OBVP

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <vector>
#include <complex>

#include "data_type.h"

template<typename State, typename T>
class MinAccOBVP {
    public:
        MinAccOBVP() = default;
        ~MinAccOBVP() = default;

        T min_acc_obvp(State start_ptr, State start_vel, State target_ptr, State target_vel) {
            T optimal_cost;
            // TODO implement your logic here
            return optimal_cost;
        }

    private:
        
};

#endif //MIN_ACC_OBVP