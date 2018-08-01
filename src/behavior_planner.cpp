#include "behavior_planner.h"
#include "behavior_planner_costs.h"
#include <functional>

using namespace std;

Vehicle::Vehicle(vector<double> ego_state)
{
	double acc_s = ego_state[2];
	if(ego_state[1] == 0)
	{
		double velocity = min(MAX_ACCELERATION * dt, MAX_SPEED);
		acc_s = (velocity - 0)/dt;
	}
	ego_state[2] = acc_s;
	vehicles[ego_id] = ego_state;
}

Vehicle::~Vehicle(){}

FSM::FSM() 
{
	current_state = "KL";//Initial value for state - Keep Lane
	state_graph["KL"] = {"KL","PLCL","PLCR"};
	state_graph["PLCL"] = {"KL","LCL"};
	state_graph["LCL"] = {"KL","PLCL"};//back to back lane change can be done as a possible next state of lane change left is prepare lane change left.
	state_graph["PLCR"] = {"KL","LCR"};
	state_graph["LCR"] = {"KL","PLCR"};	
}
FSM::~FSM() {}

void Vehicle::predict()
{
	// cout<<"in predict, vehicle id 0 d value - "<<vehicles.find(0)->second[3]<<endl;
	map<int, vector<double>>::iterator it = vehicles.begin();

	//Leaving the ego vehicle's state unchanged as it will be decided by behavior planner.
	for(it; it != vehicles.end(); ++it)
	{
		if(it->first == ego_id) continue;
		vector<double> vehicle_state = it->second;
		vector<double> update = {vehicle_state[1] * dt + 0.5 * vehicle_state[2] *dt*dt, vehicle_state[2] * dt, 0, vehicle_state[4] * dt + 0.5 * vehicle_state[5] * dt *dt, vehicle_state[5] * dt, 0};//Although the traffic vehicles are assumed to be moving at a constant speed. I have included this to tinker with the code after the project.
		std::transform(update.begin(), update.end(), vehicle_state.begin(), update.begin(), plus<double>());

		for(int i = 0; i<update.size(); i++) if(fabs(update[i]) < 0.0001) update[i] = 0.0;//Approximating the extremely small floating point values to zero.
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

vector<double> Vehicle::get_kinematics(Vehicle &vehicle, int target_lane)
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
	double d_dot_dot = current_kinematics[5];
	double new_velocity;

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
		new_velocity = 0.0;
		s_dot_dot = 0.0;
	}
	else s_dot_dot = (new_velocity - s_dot)/dt;

	s += new_velocity * dt + 0.5 * s_dot_dot * dt * dt;
	// d+= d_dot * dt + 0.5 * s_dot_dot * dt *dt;//D update is not necessary as behavior planner is used to predict the end states not the trajctory which means the vehicle will always be heading towards the lane direction. 
	new_kinematics = {s, new_velocity, s_dot_dot, d, d_dot, d_dot_dot};
	return new_kinematics;
}

void FSM::find_goal_pose(Vehicle &vehicle, string start_state, string goal_state)
{
	int target_lane;
	target_lane = (vehicle.vehicles.find(ego_id)->second[3])/4 + stoi_state.find(goal_state)->second;
	
	if(target_lane < d_min || target_lane > d_max) return;

	vector<double> new_lane_kinematics = vehicle.get_kinematics(vehicle, target_lane);
	if(goal_state.compare("KL") == 0)//KEEP LANE
	{
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id, new_lane_kinematics));
	}

	else if(goal_state.compare("PLCL") == 0|| goal_state.compare("PLCR") == 0)//PREPARE LANE CHANGE LEFT
	{
		vector<double> current_lane_new_kinematics = vehicle.get_kinematics(vehicle, vehicle.vehicles.find(ego_id)->second[3]/4);
		vector<double> best_kinematics = current_lane_new_kinematics;

		if(vehicle.find_vehicle_behind(target_lane) == -1)
		{
			if(current_lane_new_kinematics[1] > new_lane_kinematics[1]) best_kinematics = new_lane_kinematics;
			best_kinematics[3] = current_lane_new_kinematics[3];//keeping the lane after the prediction same as the initial lane.
		}	
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id, best_kinematics));	
	}

	else if(goal_state.compare("LCL")== 0 ||  goal_state.compare("LCR") == 0)//Change lane.
	{
		//Checking if lane change is possible or not.
		map<int,vector<double>>::iterator it = vehicle.predictions.begin();
		for(it; it != vehicle.predictions.end(); ++it)
		{
			// If another vehicle is already at the predicted position of the ego vehicle, then abort the FSM state change.
			if(vehicle.predictions.find(it->first)->second[0] == new_lane_kinematics[0]) return;
		}		
		new_lane_kinematics[3] = vehicle.vehicles.find(ego_id)->second[3] + stoi_state.find(goal_state)->second * 4.0;
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id,new_lane_kinematics));
	}
}

vector<vector<double>> FSM::next_state(Vehicle &vehicle, string current_state)
{
	vector<string> successors = state_graph.find(current_state)->second;
	map<string, double>costs;//Map for storing costs for all the trajectories.
	double min_cost = MAX_COST;
	string min_cost_state;
	vector<double> min_cost_trajectory;
	vector<vector<double>> trajectory;
	trajectory.push_back(vehicle.vehicles.find(ego_id)->second);

	vehicle.predict();

	for(int i = 0; i<successors.size(); i++)
	{	
		string goal_state = successors[i];
		find_goal_pose(vehicle, current_state, goal_state);
		double total_cost = calculate_cost(vehicle);
		if(total_cost < min_cost) 
		{
			min_cost_state = goal_state;
			min_cost = total_cost;
			min_cost_trajectory = vehicle.predictions.find(ego_id)->second;
		}
		costs[goal_state] = total_cost;
		vehicle.predictions.erase(ego_id);
	}
	trajectory.push_back(min_cost_trajectory);
	return trajectory;
}
		

// Predict the next state for ego vehicle using find_goal_pose for every successor and then find the cost for this movement using cost functions. 

// Problem - Max acceleration reached.
// TO DO:
// 0. Implement solution when s_dot != 0.
// 1. There is no boundation on acceleration and jerk in the trajectory generated. Create multiple trajectories and choose the one with minimum average acceleration and jerk. Bound both AccT and AccN.
// 2. Update the traffic vehicle selection search distance parameter to 100 meters. Add condition when vehicle.vehicles is empty as move with max speed in the same lane.