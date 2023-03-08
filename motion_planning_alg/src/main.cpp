#include <root_finder.h>
#include <minimum_acceleration_obvp.h>
#include <data_type.h>

template <typename T>
std::ostream&  operator<<(std::ostream& os, const std::vector<std::complex<T>>& v) { 
    os << "[";
    for (int i = 0; i < v.size(); ++i) { 
        os <<"( "<< v[i].real() << "," << v[i].imag() << "), "; 
    }
    os << "]\n";
    return os; 
}

int main()
{
    //setting coefficient of polynomial
    // 9x^4+ 34x^3 + 9 =0
    Eigen::VectorXd coeffs(5);
    coeffs(4) = 9;
    coeffs(3) = 34;
    coeffs(2) = 0;
    coeffs(1) = 0;
    coeffs(0) = 9;

    Eigen::Matrix4d A;
    A << -coeffs(3)/coeffs(4),-coeffs(2)/coeffs(4),-coeffs(1)/coeffs(4),-coeffs(0)/coeffs(4),
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0;

    RootFinder<double> root_finder;
    std::vector<std::complex<double>> eigen_values = root_finder.calculateEigenValuesOfMatrix(A);
    std::vector<std::complex<double>> roots = root_finder.calculateRootsOfPoly(coeffs);

    std::cout<< "Eigen values of matrix A: " << std::endl;
    std::cout<< eigen_values << std::endl;

    std::cout<< "Root of the polynomial: " << std::endl;
    std::cout<< roots << std::endl;

    // MinAccOBVP<Vec3d, double> min_acc_obvp;
    // Vec3d start_p = {3, 4, 5};
    // Vec3d end_p = {30, 40, 8};
    // Vec3d start_v = {0, 0, 0};
    // Vec3d end_v = {0, 0, 0};
    // double optimal_cost = min_acc_obvp.min_acc_obvp(start_p, start_v, end_p, end_v);

    return 0;
}