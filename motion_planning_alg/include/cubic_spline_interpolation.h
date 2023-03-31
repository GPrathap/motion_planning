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
#include <cnpy.h>
#include "data_type.h"
#include "motion_model.h"

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

class Spline2D
{
  private:
    Spline1D sx, sy;

  public:
    Spline2D()= default;
    ~Spline2D() = default;

    void initTrajectory(const vec1D& x_in, const vec1D& y_in);
    vec1D calcKnots(vec1D x, vec1D y);
    void calcPosition(double &x, double &y, double t);
    double calcCurvature(double t);
    double calcYaw(double t);
    double getKnotsLast();
    void calculateReferenceTrajectory(vec1D x, vec1D y, double ds);
    void saveVector(vec1D& path, std::string file_name);
    void calculateTrackingError(const double x, const double y, 
									double& e, double& k, double& yaw, int& s);
    robot_state getState(int index);
    
    inline double normalizedAngle(double theta){
      while(theta> M_PI){
        theta -= 2.0*M_PI;
      }
      while(theta < -M_PI){
        theta += 2.0*M_PI;
      }
      return theta;
    }

    vec1D x, y, knots, dknots, rx, ry, ryaw, rk;
};

#endif
