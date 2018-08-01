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
	   0, 1, 2*T, 3*pow(T,2), 4*pow(T,2), 5*pow(T,4),
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
			if(abs(coeffs[i][j]) < 0.0001) coeffs[i][j] = 0.0;
		}
	}
	return coeffs;
}