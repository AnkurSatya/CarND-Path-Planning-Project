#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include <map>

using namespace std;

static int ego_id = -1;
static int d_min = 0;
static int d_max = 2;
static int NUM_LANES = 3;
static map<string, int> stoi_state = {{"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"PLCR", 1}, {"LCR", 1}};
static double dt = 0.5;//500ms
static double SEARCH_DISTANCE= 50.0;//(in meters) Search distance for the vehicles in a lane. It decides whether to take a vehicle in consideration or not.
static double BUFFER_DISTANCE = 3.0;//(in meters) Distance to maintain between the vehicles along the heading.
static double MAX_SPEED = (50.0 * 1.60934 * 5.0/18.0);//in m/s.
static double MAX_ACCELERATION = 10;//in m/s^2.
static double MAX_COST = 1e10;

class Vehicle
{
public:
	//Constructor is used for variable initializing and configuring the ego vehicle state.
	// Vehicle(double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot);
	Vehicle(vector<double> ego_state);
	virtual ~Vehicle();
	//includes id, s, s_dot,s_dot_dot, d, d_dot, d_dot_dot.
	map<int, vector<double>> vehicles;
	// contains the predicted state of the all the vehicles except for the ego vehicle.
	map<int, vector<double>> predictions;

	// predicts t0e states of all the vehicles except for the ego vehicle.
	void predict();
	int find_vehicle_ahead(int target_lane);
	int find_vehicle_behind(int target_lane);
	vector<double> get_kinematics(Vehicle &vehicle, int target_lane);
};

class FSM
{
	//Following states are included - Keep Lane, Prepare Left lane change, left lane change, Prepare Right lane change, right lane change.
public:
	FSM();
	virtual ~FSM();

	//States and their possible next states.
	map<string, vector<string>> state_graph;
	
	string current_state;

	//returns the trajectory for transition to the next state to the Trajectory Planner Module.
	vector<vector<double>> next_state(Vehicle &vehicle, string current_state);

	//Generates trajectory for transition to a input state. 
	//Returns within the next_state function for cost evaluation.
	// It should include the present state as the first state.
	void find_goal_pose(Vehicle &vehicle, string start_state, string goal_state);

	//Evaluates the cost for transition to the next state.
	double evaluate_cost();


};

#endif

// Important Points:
// 1. The simulator gives the controls to the vehicle every 20ms. It gives the controls on the basis of the values in the waypoints fed to it by the code.
// 2. So, the number of waypoints we should generate should depend on our code cycle time/20ms.
// 3. When generating the trajectory, use dt = 0.02 to generate a waypoint. This is not necessary.

// Implement:
// 0. Change lane only when the vehicle ahead is moving slower than 50MPH.
// 1. Figure out how to store the acceleration value after one interation.
// 2. Add acceleration to the state.
// 3. Initialize intial acceleration carefully, so as to avoid max jerk constraint.

//How to determine the acceleration after an iteration.
//1. We have the car's current vehicle and position.
//2. The first coordinate in the previous_waypoints is the car's next position. Calculate the acceleration using this information.
//3. repeat this, until you reach the previous_waypoints end coordinate.