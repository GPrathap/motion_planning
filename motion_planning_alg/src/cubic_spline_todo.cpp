#include <cubic_spline_interpolation.h>

MatXd Spline1D::calculateMatrixA(vec1D h)
{
	MatXd A = MatXd::Zero(n, n);
	// TODO calculate the matrix A 
	return A;
}

MatXd Spline1D::calculateVectorb(vec1D h){
	MatXd B = MatXd::Zero(n, 1);
	// TODO calculate the vector b 
	return B;
}

int Spline1D::search_index(double p){
	// TODO given time, return cloest index of the time (t)
}

void Spline1D::init(vec1D t_in, vec1D y_in){
	t = t_in;
	y = y_in;
	n = t.size();
	vec1D h;
	for (int i = 1; i < n; i++)
		h.push_back(t[i] - t[i - 1]);
	
	MatXd A = calculateMatrixA(h);
	MatXd B = calculateVectorb(h);
	MatXd C = A.inverse() * B;

	// TODO estimate values for a,b, c, and d 
}

double Spline1D::cal(double t_index){
	// TODO given time, check the given time is within
	// the range of the constructed spline, if not return NOT_DEFINED else 
	// return the position at time t_index. Hint; you may use search_index 
	// function to get index  
	double result;
	return result;
}

double Spline1D::cald(double t_index)
{
	// TODO given time, check the given time is within
	// the range of the constructed spline, if not return NOT_DEFINED else 
	// return the velocity at time t_index. Hint; you may use search_index 
	// function to get index 
	double result;
	return result;
}

double Spline1D::caldd(double t_index) // y_doubleDot at given x
{
	// TODO given time, check the given time is within
	// the range of the constructed spline, if not return NOT_DEFINED else 
	// return the acceleration at time t_index. Hint; you may use search_index 
	// function to get index 
	double result;
	return result;
}