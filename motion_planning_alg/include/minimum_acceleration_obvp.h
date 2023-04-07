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
        // initial positions, velocities, acceleration that we need to calculate the optimal cost 
            double px0, pxf, vx0, vxf, py0, pyf, vy0, vyf, pz0, pzf, vz0, vzf;

            //initial positions
            px0 = start_ptr(0);  py0 = start_ptr(1);  pz0 = start_ptr(2);

            //final positions
            pxf = target_ptr(0);  pyf = target_ptr(1);  pzf = target_ptr(2);

            //initial and final speeds
            vxf = target_vel(0);  vx0 = start_vel(0);   vyf = target_vel(1);
            vy0 = start_vel(1);   vzf = target_vel(2);  vz0 = start_vel(2);
        
        // Coefficients for the polynomial
            double a4, a3, a2, a1, a0;
            a4 = 1;
            a3 = 0;
            a2 = -4*
                (std::pow(vx0, 2) + (vx0*vxf) + std::pow(vxf,2) + std::pow(vy0,2) +
                (vy0*vyf) + std::pow(vyf,2) +
                std::pow(vz0,2) + (vz0*vzf) + std::pow(vzf,2)
            );
            a1 = -24 * ( (px0*vx0) + (px0*vxf) - (pxf*vx0) -(pxf*vxf) + 
                         (py0*vy0) + (py0*vyf) - (pyf*vy0) -(pyf*vyf) + 
                         (pz0*vz0) + (pz0*vzf) - (pzf*vz0) -(pzf*vzf)
                 
            );
            a0 = -36 * ( std::pow(px0,2) - (2*px0*pxf) +
                         std::pow(pxf,2) + std::pow(py0,2) - (2*py0*pyf) +
                         std::pow(pyf,2) +std::pow(pz0,2) - (2*pz0*pzf) +
                         std::pow(pzf,2)
                
            );
        // now we want to find the eigenvalues for A and choose the biggest eigenValue (real part)
            Eigen::Matrix4d A;
            A << 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1,
                -a0, -a1, -a2, -a3;

            /*std::cout<< "a_0: " << a0 << std::endl;
            std::cout<< "a_1: " << a1 << std::endl;
            std::cout<< "a_2: " << a2 << std::endl;
            std::cout<< "a_3: " << a3 << std::endl;
            std::cout<< "a_4: " << a4 << std::endl;*/
            
        // now we find the roots of the matrix    
            RootFinder<double>root_finder;
            std::vector<std::complex<double>> eigen_values = root_finder.calculateEigenValuesOfMatrix(A);

            for (int i=0; i < eigen_values.size(); ++i){
                if (eigen_values[i].real() > optimal_cost){
                    optimal_cost = eigen_values[i].real();
                }
            }

            std::cout<< std::endl << "optimal_cost: " << optimal_cost << std::endl;
            return optimal_cost;
        }

    private:
        
};

#endif //MIN_ACC_OBVP