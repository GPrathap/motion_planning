#include <cubic_spline_interpolation.h>

MatXd Spline1D::calculateMatrixA(vec1D h)
{
	MatXd A = MatXd::Zero(n, n);
	A(0, 0) = 1.0;
	for (int i = 0; i < n - 1; i++)
	{
		if (i != n - 2)
			A(i + 1, i + 1) = 2.0 * (h[i] + h[i + 1]);
		A(i + 1, i) = h[i];
		A(i, i + 1) = h[i];
	}
	A(0, 1) = 0.0;
	A(n - 1, n - 2) = 0.0;
	A(n - 1, n - 1) = 1.0;
	return A;
}

MatXd Spline1D::calculateVectorb(vec1D h){
	MatXd B = MatXd::Zero(n, 1);
	for (int i = 0; i < n - 2; i++)
		B(i + 1, 0) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
	return B;
}

int Spline1D::search_index(double p){
	auto it = upper_bound(t.begin(), t.end(), p);
	if (it == t.end())
	{
		return t.size() - 2;
	}
	else
	{
		return it - t.begin() - 1;
	}
}

void Spline1D::init(vec1D t_in, vec1D y_in){
	t = t_in;
	y = y_in;
	n = t.size();
	vec1D h;
	for (int i = 1; i < n; i++)
		h.push_back(t[i] - t[i - 1]);
	a = y;

	MatXd A = calculateMatrixA(h);

	MatXd B = calculateVectorb(h);

	MatXd C = A.inverse() * B;

	for (int i = 0; i < C.rows(); ++i)
		c.push_back(C(i, 0));
	for (int i = 0; i < n - 1; i++)
	{
		d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
		double tb = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
		b.push_back(tb);
	}
}

double Spline1D::cal(double t_index){
	if (t.size() == 0)
	{
		return NOT_DEFINED;
	}
	else if (t_index < t[0])
	{
		return NOT_DEFINED;
	}
	else if (t_index > t[n - 1])
	{
		return NOT_DEFINED;
	}
	int i = search_index(t_index);
	double dt = t_index - t[i];
	if (i > n - 1)
	{
		return NOT_DEFINED;
	}
	double result = a[i] + b[i] * dt + c[i] * dt * dt + d[i] * dt * dt * dt;
	return result;
}

double Spline1D::cald(double t_index)
{
	if (t_index < t[0])
	{
		return NOT_DEFINED;
	}
	else if (t_index > t[n - 1])
	{
		return NOT_DEFINED;
	}
	int i = search_index(t_index);
	double dt = t_index - t[i];
	double result = b[i] + 2 * c[i] * dt + 3 * d[i] * dt * dt;

	return result;
}

double Spline1D::caldd(double t_index){
	if (t_index < t[0])
		return NOT_DEFINED;
	else if (t_index > t[n - 1])
		return NOT_DEFINED;

	int i = search_index(t_index);
	double dt = t_index - t[i];
	double result = 2 * c[i] + 6 * d[i] * dt;
	return result;
}

void Spline2D::initTrajectory(const vec1D& x_in, const vec1D& y_in){
      x = x_in;
      y = y_in;
      knots = calcKnots(x, y);
      sx.init(knots, x);
      sy.init(knots, y);
	  return;
}

vec1D Spline2D::calcKnots(vec1D x, vec1D y) {
	vec1D dx;
	for (unsigned int i = 1; i < x.size(); i++)
	{
		dx.push_back(x[i] - x[i - 1]);
	}
	vec1D dy;
	for (unsigned int i = 1; i < x.size(); i++)
	{
		dy.push_back(y[i] - y[i - 1]);
	}
	dknots.clear();
	for (unsigned int i = 0; i < dx.size(); i++)
	{
		double temp;
		temp = sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
		dknots.push_back(temp);
	}

	vec1D knots;
	knots.push_back(0);

	for (unsigned int i = 0; i < dknots.size(); i++)
	{
		knots.push_back(knots.back() + dknots[i]);
	}

	return knots;
}

void Spline2D::calcPosition(double &x, double &y, double t){
	x = sx.cal(t);
	y = sy.cal(t);
}

double Spline2D::calcCurvature(double t){
	double dx = sx.cald(t);
	double ddx = sx.caldd(t);
	double dy = sy.cald(t);
	double ddy = sy.caldd(t);
	double k = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
	return k;
}

double Spline2D::calcYaw(double t){
	double dx = sx.cald(t);
	double dy = sy.cald(t);
	if (dx == 0)
	{
		return 1.57 * (dy > 0);
	}
	double yaw = atan2(dy, dx);
	return yaw;
}



double Spline2D::getKnotsLast(){
	return knots.back();
}

void Spline2D::saveVector(vec1D& path, std::string file_name){
  std::vector<double> edges;
  for (int k = 0; k < (int)path.size(); k++)
  {
    edges.push_back(path[k]);
  }
  std::string storing_location = file_name;
  if (edges.size() > 0)
  {
    cnpy::npy_save(storing_location, &edges[0],{(unsigned int)path.size(), (unsigned int)1, 1}, "w");
  }
  else
  {
    std::cout << "No data to save " << storing_location << std::endl;
  }
  return;
}

// generates the Spline2D with points along the spline at distance = ds, also returns
// yaw and curvature
void Spline2D::calculateReferenceTrajectory(vec1D x, vec1D y, double ds){
	initTrajectory(x, y);
	vec1D s;
	double sRange = getKnotsLast();
	double sInc = 0;
	while (1)
	{
		if (sInc >= sRange)
		{
			break;
		}
		s.push_back(sInc);
		sInc = sInc + ds;
	}
	rx.resize(s.size());
	ry.resize(s.size());
	ryaw.resize(s.size());
	rk.resize(s.size());
	for (int i = 0; i < s.size(); i++)
	{
		double ix, iy;
		calcPosition(ix, iy, s[i]);
		rx[i] = ix;
		ry[i] = iy;
		ryaw[i] = calcYaw(s[i]);
		rk[i] = calcCurvature(s[i]);
	}
	return;
}

robot_state Spline2D::getState(int index){
	robot_state current_state = {rx[index], ry[index], ryaw[index]};
	return current_state;
}

void Spline2D::calculateTrackingError(const double x, const double y, 
									double& e, double& k, double& yaw, int& s){
	vec1D d_x;
	vec1D d_y;
	vec1D d;

	d_x.resize(rx.size());
	d_y.resize(rx.size());
	d.resize(rx.size());

	for(int i=0; i<rx.size(); i++){
		d_x[i] = rx[i]-x;
		d_y[i] = ry[i]-y;
		d[i] = std::sqrt(d_x[i]*d_x[i] + d_y[i]*d_y[i]);
	}

	s = std::distance(d.begin(), std::min_element(d.begin(), d.end()));
	yaw = ryaw[s];
	k = rk[s];
	double angle = normalizedAngle(yaw - atan2(d_y[s], d_x[s]));
	e = d[s];
	e = (angle < 0)? e*-1: e;
}
