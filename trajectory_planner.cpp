#include "trajectory_planner.h"

generate_trajectory::generate_trajectory(double time_for_maneuver)
{
	T = time_for_maneuver;
}

generate_trajectory::~generate_trajectory(){}
vector<vector<double>> generate_trajectory::generate_JMT(vector<vector<double>> trajectory)
{
	vector<double> initial_boundary_conditions = trajectory[0];
	vector<double> final_boundary_conditions = trajectory[1];
	MatrixXd A(6,6);
	VectorXd B(6);
	VectorXd X(6);
	vector<vector<double>> coeffs;

	// //Generating JMT for 's'.
	A<<1, 0, 0, 0, 0, 0,
	   0, 1, 0, 0, 0, 0,
	   0, 0, 2, 0, 0, 0,
	   1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
	   0, 1, 2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
	   0, 0, 2, 6*T, 12*pow(T,2), 20*pow(T,3);

	B<<initial_boundary_conditions[0], initial_boundary_conditions[1], initial_boundary_conditions[2], final_boundary_conditions[0], final_boundary_conditions[1], final_boundary_conditions[2];

	X = A.colPivHouseholderQr().solve(B);

	{vector<double> coeffs_sub(X.data(), X.data() + X.rows() * X.cols());
	coeffs.push_back(coeffs_sub);}

	//Generating JMT for 'd'.
	//A remains the same.
	B<<initial_boundary_conditions[3], initial_boundary_conditions[4], initial_boundary_conditions[5],
		final_boundary_conditions[3], final_boundary_conditions[4], final_boundary_conditions[5];

	X = A.colPivHouseholderQr().solve(B);
	vector<double> coeffs_sub(X.data(), X.data() + X.rows()*X.cols());
	coeffs.push_back(coeffs_sub);
	
	for(int i = 0; i<coeffs.size();i++)
	{
		for(int j = 0; j<coeffs[0].size(); j++)
		{
			if(abs(coeffs[i][j]) < 0.000001) coeffs[i][j] = 0.0;
		}
	}
	return coeffs;
}

vector<vector<double>> generate_trajectory::generate_JMT_waypoints(vector<vector<double>> coeffs, double T, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	//T represents the time for completing this trajectory.
	double s = 0.0;// s is not initialized with start_s because the coeffs already contain that information.
	double d = 0.0;
	for(int i = 0; i<coeffs[0].size(); i++)
	{
		s+= coeffs[0][i] * pow(T,i);
		d+= coeffs[1][i] * pow(T,i);
	}
	vector<double> XY = getXY(s,d,maps_s, maps_x, maps_y);
	return {XY,{s,d}};
}

vector<double> generate_trajectory::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	//This is a very poorly designed mathematical formulation. The approximation is not fixed and increases with the curvature, map waypoint and vehicle position. Problem can be solved by using splines to generate more waypoints.
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}