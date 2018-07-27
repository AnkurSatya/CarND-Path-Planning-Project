#include "behavior_planner.h"
#include <algorithm>
#include <functional>

using namespace std;

Vehicle::Vehicle(double s, double s_dot, double s_dot_dot, double d, double d_dot, double d_dot_dot)
{
	double acc_s = s_dot_dot;
	if(s_dot == 0)
	{
		double velocity = min(MAX_ACCELERATION * dt, MAX_SPEED);
		acc_s = (velocity - 0)/dt;
	}

	vehicles[ego_id] = {s,s_dot, acc_s, d,d_dot, d_dot_dot};
}

Vehicle::~Vehicle(){}

FSM::FSM() 
{
	current_state = "KL";//Initial value for state - Keep Lane
	state_graph["KL"] = {"KL","PLCL","PLCR"};
	state_graph["PLCL"] = {"KL","PLCL","LCL"};
	state_graph["LCL"] = {"KL","PLCL"};//back to back lane change can be done as a possible next state of lane change left is prepare lane change left.
	state_graph["PLCR"] = {"KL","PLCR","LCR"};
	state_graph["LCR"] = {"KL","PLCR"};	
}
FSM::~FSM() {}

void Vehicle::predict()
{
	map<int, vector<double>>::iterator it = vehicles.begin();
	++it;

	//Leaving the ego vehicle's state unchanged as it will be decided by behavior planner.
	for(it; it != vehicles.end(); ++it)
	{
		vector<double> update = {it->second[1] * dt, 0, 0, it->second[4] * dt, 0};
		std::transform(update.begin(), update.end(), it->second.begin(), update.begin(), plus<double>());
		predictions.insert(pair<int, vector<double>> (it->first, update));
	}
}

int Vehicle::find_vehicle_ahead(int target_lane)
{
	int id = ego_id;
	map<int, vector<double>>::iterator it = vehicles.begin();
	++it;

	for(it; it != vehicles.end(); ++it)
	{
		vector<double> new_vehicle_state = predictions.find(it->first)->second;
		if(int(new_vehicle_state[3]/4) == target_lane) 
		{
			if(new_vehicle_state[0] - vehicles.find(ego_id)->second[0] <= SEARCH_DISTANCE && new_vehicle_state[0] > vehicles.find(ego_id)->second[0]) id = it->first;//checking for the vehicle in the target lane.
		}
	}
	return id;
}

int Vehicle::find_vehicle_behind(int target_lane)
{
	int id = ego_id;
	map<int, vector<double>>::iterator it = vehicles.begin();
	++it;

	for(it; it != vehicles.end(); ++it)
	{
		vector<double> new_vehicle_state = predictions.find(it->first)->second;
		if(int(predictions.find(it->first)->second[3]/4) == target_lane) 
		{
			if(vehicles.find(ego_id)->second[0] - new_vehicle_state[0] <= SEARCH_DISTANCE && new_vehicle_state[0] < vehicles.find(ego_id)->second[0]) id = it->first;//checking for the vehicle in the target lane.
		}
	}
	return id;
}

vector<double> Vehicle::get_kinematics(Vehicle vehicle, int target_lane)
{
	int vehicle_ahead_id = find_vehicle_ahead(target_lane);
	int vehicle_behind_id = find_vehicle_behind(target_lane);

	vector<double> new_kinematics;
	vector<double> current_kinematics = vehicle.vehicles.find(ego_id)->second;

	double s = current_kinematics[0];
	double s_dot = current_kinematics[1];
	double s_dot_dot = current_kinematics[2];
	double d = current_kinematics[3];
	double d_dot = current_kinematics[4];
	double new_velocity;

	//Implementing a condition for no lane change if the vehicle ahead is moving at a velocity >= MAX_SPEED.
	if(vehicle.predictions.find(vehicle_ahead_id)->second[1] >= MAX_SPEED) return {};

	//If there is traffic, move with the traffic's speed.
	if(vehicle_ahead_id != -1)
	{
		vector<double> vehicle_ahead_kinematics = vehicle.predictions.find(vehicle_ahead_id)->second;
		if(vehicle_behind_id != -1) new_velocity = min(vehicle_ahead_kinematics[1], MAX_SPEED);
		else
		{
			new_velocity = (vehicle_ahead_kinematics[0] - s - BUFFER_DISTANCE)/dt + vehicle_ahead_kinematics[1] - 0.5*s_dot_dot*dt*dt;//The acceleration term is added to negate the effect of the acceleration at the previous timestamp on the new velocity.
			new_velocity = min(min(MAX_ACCELERATION*dt + s_dot, new_velocity), MAX_SPEED);
		}
	}
	else//keeps a check on both max acceleration and max speed.
	{	
		new_velocity = min(MAX_ACCELERATION * dt, MAX_SPEED);
	}

	if(new_velocity < 0) 
	{
		new_velocity = 0;
		s_dot_dot = 0;
	}
	else s_dot_dot = (new_velocity - s_dot)/dt;

	s += new_velocity * dt + 0.5 * s_dot_dot * dt * dt;
	new_kinematics = {s, new_velocity, s_dot_dot, d, d_dot};
	return new_kinematics;
}

void FSM::find_goal_pose(Vehicle &vehicle, string start_state, string goal_state)
{
	int target_lane;

	target_lane = (vehicle.vehicles.find(ego_id)->second[3]/4) + stoi_state.find(goal_state)->second;
	// cout<<"target_lane - "<<target_lane<<'\n';
	if(target_lane < d_min || target_lane > d_max) return;

	if(goal_state.compare("KL") == 0)//KEEP LANE
	{
		// cout<<"target_lane - "<<target_lane<<'\n';
		vector<double> new_kinematics = vehicle.get_kinematics(vehicle,target_lane);
		// cout<<"New s - "<<new_kinematics[0]<<"New s_dot - "<<new_kinematics[1]<<'\n';
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id, new_kinematics));
	}

	else if(goal_state.compare("PLCL") == 0 || goal_state.compare("PLCR") == 0)//PREPARE LANE CHANGE LEFT
	{
		vector<double> current_kinematics = vehicle.vehicles.find(ego_id)->second;
		vector<double> new_kinematics = vehicle.get_kinematics(vehicle, target_lane);

		if(vehicle.find_vehicle_behind(target_lane) != -1)
		{
			map<int, vector<double>>::iterator it = vehicle.predictions.begin();
			for(it; it != vehicle.predictions.end(); it++)
			{
				//If another vehicle is already at the predicted position of the ego vehicle, then abort the FSM state change.
				if(vehicle.predictions.find(it->first)->second[0] == new_kinematics[0]) 
				{
					vector<double> ego_n = current_kinematics;//ego updated state.
					ego_n[0]+= ego_n[1]*dt + 0.5*ego_n[2]*dt*dt;
					ego_n[1]+= ego_n[2]*dt;

					vehicle.predictions.insert(pair<int, vector<double>>(ego_id, ego_n));
					return;
				}
			}
		}

		else
		{
			vector<double> current_lane_new_kinematics = vehicle.get_kinematics(vehicle, current_kinematics[3]/4);
			vector<double> best_kinematics = current_lane_new_kinematics;

			if(current_lane_new_kinematics[1] > new_kinematics[1]) best_kinematics = new_kinematics;

			best_kinematics[3] = current_kinematics[3]/4;//keeping the lane after the prediction same as the initial lane.
			vehicle.predictions.insert(pair<int, vector<double>>(ego_id, best_kinematics));
		}		
	}
}

vector<vector<double>> FSM::next_state(Vehicle &vehicle, string current_state)
{
	vector<string> successors = state_graph.find(current_state)->second;
	vehicle.predict();
	map<int, double>costs;//Map for storing costs for each trajectory.
	for(int i =0; i<successors.size(); i++)
	{
		string start_state = current_state;
		string goal_state = successors[i];
		find_goal_pose(vehicle, start_state, goal_state);
		// vector<double> predicted = vehicle.predictions.find(ego_id)->second;
		// cout<<"Predictions:"<<'\n'<<"S - "<<predicted[0]<<" S_dot "<<predicted[1]<<" D "<<int(predicted[3]/4)<<'\n';

		//Evaluate cost now by passing vehicle object as reference. First check if an object with ego_id exists or not.

		//
		vehicle.predictions.erase(ego_id);
	}
	//CHANGE THE STATE OF THE EGO VEHICLE.

	vector<vector<double>> ret = {{1,2,3}};
	return ret;
}
		

// Predict the next state for ego vehicle using find_goal_pose for every successor and then find the cost for this movement using cost functions. 

// TO DO:
// 0. Decide on Code exectution flowchart.
// 1. Check the output of the code written so far by giving inputs from the simulator.
// 2. Implement cost functions.
// 3. Decide how to go from KL-> LCL in one code cycle.