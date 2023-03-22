#ifndef CUBIC_SPLINE_INTERPOLATION
#define CUBIC_SPLINE_INTERPOLATION
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <vector>
#include <complex>

#include "data_type.h"

/************************
For a given set of points (t_i, y_i),i=0,...,n, generates cubic splines 
s_i(t) = y_i(== a_i) + b_i*(t - t_i) + c_i*(t - t_i)^2 + d_i*(t - t_i)^3
************************/
class Spline1D
{
  private:
    vec1D a, b, c, d;
    vec1D t, y;
    int n;
    const double NOT_DEFINED = -1e4;
    MatXd calculateMatrixA(vec1D h);
    MatXd calculateVectorb(vec1D h);
    int search_index(double p);

  public:
    void init(vec1D t_in, vec1D y_in);
    double cal(double t_index);
    double cald(double t_index);
    double caldd(double t_index);
};

#endif  // CUBIC_SPLINE_INTERPOLATION
