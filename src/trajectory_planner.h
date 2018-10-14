#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Eigen"
#include "constants.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class generate_trajectory
{
public:
	double T;

	generate_trajectory(double time);
	virtual ~generate_trajectory();
	vector<vector<double>> generate_JMT(vector<vector<double>> trajectory);
	vector<vector<double>> generate_JMT_waypoints(vector<vector<double>> coeffs, double T, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
};

#endif