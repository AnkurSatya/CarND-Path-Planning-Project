#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Eigen"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class generate_trajectory
{
public:
	double T;

	generate_trajectory(double time_for_maneuver);
	virtual ~generate_trajectory();
	vector<vector<double>> generate_JMT(vector<vector<double>> trajectory);
};

#endif