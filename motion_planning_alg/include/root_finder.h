#ifndef ROOT_FINDER
#define ROOT_FINDER

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <vector>
#include <complex>

template<typename T> 
class RootFinder {
    public:
        RootFinder() = default;
        ~RootFinder() = default;

        inline std::vector<std::complex<T>> calculateEigenValuesOfMatrix(Eigen::MatrixXd A){
            Eigen::EigenSolver<Eigen::MatrixXd> ES(A);
            std::vector<std::complex<T>> eigen_values;
            for(int i =0;i<ES.eigenvalues().rows();++i){
                eigen_values.push_back(ES.eigenvalues()[i]);
            }
            return eigen_values;
        };

        inline std::vector<std::complex<T>> calculateRootsOfPoly(Eigen::VectorXd coeffs){
            Eigen::PolynomialSolver<T, Eigen::Dynamic> solver;
            solver.compute(coeffs);
            const typename Eigen::PolynomialSolver<T, Eigen::Dynamic>::RootsType & r = solver.roots();
            std::vector<std::complex<T>> roots;
            for(int i =0;i<r.rows();++i)
            {
                roots.push_back(r[i]);
            }
            return roots;
        };

    private:
        
};

#endif //ROOT_FINDER
