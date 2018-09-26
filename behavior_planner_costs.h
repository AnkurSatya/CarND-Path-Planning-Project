#ifndef BEHAVIOR_PLANNER_COSTS_H
#define BEHAVIOR_PLANNER_COSTS_H

#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

using namespace std;

double sigmoid(double x)
{
	return (1.0 - 2.0/(1+exp(x)));
}

bool sort_by_speed (const vector<double> &first, const vector<double> &second)
{
	return first[2] > second[2];
}

//cost for not driving at the maximum possible traffic speed and not driving in the fastest lane.
double inefficiency_cost(Vehicle &vehicle, int target_lane_for_state)
{
	
	vector<double> ego_vehicle_state = vehicle.predictions.find(ego_id)->second;
	if(ego_vehicle_state[1] > MAX_SPEED) return 1.0;

	vector<double> lane_speed(NUM_LANES,-1.0);
	map<int, vector<double>>::iterator it = vehicle.predictions.begin();
	vector<vector<double>> lane_distance_speed(NUM_LANES, vector<double>(4, max_s));// Lane_id, Min distance from traffic in the lane, speed of the lane, traffic vehicle ID.

	// When evaluating the velocity of a lane, check velocity of the vehicle ahead of the ego vehicle.
	for(it; it != vehicle.predictions.end(); ++it)
	{
		// if(it->first == ego_id) continue; //To avoid the scenario where ego vehicle is not moving but still in the fastest available lane.
		// cout<<"Check 0.0"<<endl;
		// cout<<"traffic vehicle state size = "<<it->second.size()<<endl;
		if(it->first == ego_id) continue;
		vector<double> traffic_vehicle_state = it->second;
		cout<<"traffic vehicle state size = "<<traffic_vehicle_state.size()<<endl;

		int lane_id = traffic_vehicle_state[3]/4;
		
		if(lane_id <d_min || lane_id > d_max)
		{
			if(lane_id < d_min) lane_id++;
			else lane_id --;
		}

		double delta_s = traffic_vehicle_state[0] - ego_vehicle_state[0];
		cout<<"Vehicle ID, D value = "<<it->first<<", "<<traffic_vehicle_state[3]<<endl;
		if(delta_s > 0.0 && delta_s < lane_distance_speed[lane_id][1] && delta_s <= SEARCH_DISTANCE)
		{	
			lane_distance_speed[lane_id][0] = lane_id;
			lane_distance_speed[lane_id][1] = delta_s;
			lane_distance_speed[lane_id][3] = it->first;
		}
	}

	for(int i = 0; i<NUM_LANES; i++)
	{	
		if(lane_distance_speed[i][0] != max_s) 
		{
			lane_distance_speed[i][2] = vehicle.predictions.find(lane_distance_speed[i][3])->second[1];
		}
	}
	// if(lane_distance_speed[int(ego_vehicle_state[3]/4)][2] == max_s) lane_distance_speed[int(ego_vehicle_state[3]/4)][2] = MAX_SPEED;
	
	//If any lane speed is still not defined.
	for(int i = 0; i<NUM_LANES; i++) 
	{
		if(lane_distance_speed[i][2] == max_s) 
		{
			lane_distance_speed[i][2] = MAX_SPEED;
			lane_distance_speed[i][0] = i;
		}
	}
	cout<<"Printing out the lanes' velocities ....."<<endl;
	double max_speed = 0;
	for(int i = 0; i<NUM_LANES; i++)
	{
		max_speed = max(max_speed, lane_distance_speed[i][2]);
		cout<<"Lane ID, velocity ->"<<lane_distance_speed[i][0]<<", "<<lane_distance_speed[i][2]<<endl;
	}

	vector<vector<double>> lane_distance_speed_copy = lane_distance_speed;
	std::sort(lane_distance_speed_copy.begin(), lane_distance_speed_copy.end(), sort_by_speed);
	
	vector<int> lane_distance_speed_copy_index;
	lane_distance_speed_copy_index.push_back(0);
	for(int i = 0; i<NUM_LANES-1;i++)
	{
		if(lane_distance_speed_copy[i][2] - lane_distance_speed_copy[i+1][2] <= 0.2) lane_distance_speed_copy_index.push_back(i+1);
	}

	// int max_speed_lane = max_element(lane_distance_speed.begin(), lane_distance_speed.end(),sort_by_speed) - lane_distance_speed.begin();
	
	// double speed_cost = MAX_COST;
	// double lane_cost = MAX_COST;

	// for(int j = 0; j<lane_distance_speed_copy_index.size(); j++)
	// {
	// 	int max_speed_lane = j;
	// 	double max_speed = lane_distance_speed_copy[max_speed_lane][2];
	// 	if(sigmoid((max_speed - lane_distance_speed[target_lane_for_state][2])/max_speed) < speed_cost) speed_cost = sigmoid((max_speed - lane_distance_speed[target_lane_for_state][2])/max_speed);
	// 	// if(sigmoid(double(abs((max_speed_lane- target_lane_for_state)))/double(NUM_LANES)) < lane_cost) lane_cost = sigmoid(double(abs((max_speed_lane- target_lane_for_state)))/double(NUM_LANES));//cost for distance between the max speed and ego's lane.

	// 	cout<<"Maximum speed "<<max_speed<<endl;
	// 	cout<<"inefficiency cost -> "<<endl<<"        "<<" Speed cost = "<<speed_cost<<endl;
	// }

	double speed_cost = sigmoid((max_speed - lane_distance_speed[target_lane_for_state][2])/max_speed); 	
	cout<<"inefficiency cost = "<<speed_cost<<endl;
	return speed_cost;
}

double distance_from_traffic_vehicle_cost(Vehicle &vehicle, int target_lane_for_state, string goal_state, string cost_for)
{	
	//Change buffer cost. Calculate distance between the ego vehicle and the closest vehicle in the target lane for the state.
	map<int, vector<double>>::iterator it = vehicle.predictions.begin();
	int ego_predicted_lane = target_lane_for_state;

	if(cost_for.compare("STATE") == 0)
	{
		cout<<"STATE"<<endl;
		if(goal_state.compare("PLCL") || goal_state.compare("PLCR")) ego_predicted_lane = vehicle.predictions.find(ego_id)->second[3]/4;
	}

	// else if(cost_for.compare("TRAJECTORY") == 0)
	// {}

	double ego_predicted_s = vehicle.predictions.find(ego_id)->second[0];
	double min_distance_ahead = 1000;
	double min_distance_behind = 1000;
	
	for(it; it != vehicle.predictions.end(); it++)
	{
		if(it->first == ego_id) continue;
		else
		{
			if(int(it->second[3]/4) == ego_predicted_lane) 
			{
				cout<<"Checking distance from the chosen traffic vehicle = "<<it->second[0] - ego_predicted_s<<endl;
				cout<<"Vehicle Lane = "<<ego_predicted_lane<<endl;
				cout<<"Velocity of the traffic vehicle = "<<it->second[1]<<endl<<endl;
				if(it->second[0] > ego_predicted_s) min_distance_ahead = min(min_distance_ahead, it->second[0] - ego_predicted_s);
				else if(it->second[0] <= ego_predicted_s) min_distance_behind = min(min_distance_behind, ego_predicted_s - it->second[0]);
			}
		}
	}

	// if(min_distance_ahead < BUFFER_DISTANCE || min_distance_behind < BUFFER_DISTANCE) 
	// {
	// 	cout<<"Distance cost = "<<MAX_COST<<endl;
	// 	cout<<"Minimum distance ahead/behind less than BUFFER DISTANCE"<<endl;
	// 	return MAX_COST;
	// }

	// double cost_ahead = min(1.0,(min_distance_ahead - BUFFER_DISTANCE)/min_distance_ahead); //The cost = |x-3| bounded between 0 and 1.
	// double cost_behind = min(1.0, (min_distance_behind - BUFFER_DISTANCE)/min_distance_behind);
	double cost_ahead = min(1.0, BUFFER_DISTANCE/min_distance_ahead);
	double cost_behind = min(1.0, BUFFER_DISTANCE/min_distance_behind);

	double cost = (cost_ahead + cost_behind)/2.0;
	
	cout<<"Distance Costs->"<<endl<<"        "<<"Ahead Cost = "<<cost_ahead<<endl<<"        "<<"Behind Cost = "<<cost_behind<<endl;
	return cost;
}

double speed_cost(Vehicle &vehicle)
{
	if(vehicle.predictions.find(ego_id)->second[1] > MAX_SPEED) return MAX_COST;
	double cost = sigmoid((MAX_SPEED - vehicle.predictions.find(ego_id)->second[1])/MAX_SPEED);
	cout<<"Ego vehicle speed = "<<vehicle.predictions.find(ego_id)->second[1]<<endl;
	cout<<"Speed cost = "<<cost<<endl;
	return cost;
}

double acceleration_cost(Vehicle &vehicle)
{
	if(vehicle.predictions.find(ego_id)->second[2] > MAX_ACCELERATION) return MAX_COST;
	double cost = sigmoid((MAX_ACCELERATION - vehicle.predictions.find(ego_id)->second[2])/MAX_ACCELERATION); 
	cout<<"Ego vehicle acceleration = "<<vehicle.predictions.find(ego_id)->second[2]<<endl;
	cout<<"Acceleration cost = "<<cost<<endl;
	return cost;
}

double calculate_cost(Vehicle &vehicle, int target_lane_for_state, string goal_state)
{
	if(vehicle.predictions.count(ego_id) == 0) return MAX_COST;
	vector<double> weights = {25,40,10,5};

	cout<<"Individual Costs - "<<endl;
	double ineff_cost = inefficiency_cost(vehicle, target_lane_for_state);
	double distance_cost = distance_from_traffic_vehicle_cost(vehicle, target_lane_for_state, goal_state, "STATE");
	double sp_cost = speed_cost(vehicle);
	double acc_cost = acceleration_cost(vehicle);

	if(distance_cost == MAX_COST) weights[1] = 1;
	if(sp_cost == MAX_COST) weights[2] = 1;
	if(acc_cost == MAX_COST) weights[3] = 1;
	
	double total_cost = weights[0]*ineff_cost + weights[1]*distance_cost + weights[2]*sp_cost + weights[3]*acc_cost;
	return total_cost;
}

double vector_sum(double x, double y)
{
	return pow(x*x + y*y, 0.5);
}

vector<vector<double>> speed_acceleration_waypoints(vector<vector<vector<double>>> waypoints)
{
	vector<double> speed_waypoints;
	vector<double> acceleration_waypoints;
	// cout<<"Printing Speed and acceleration waypoints->"<<endl<<"Speed, "<<"Acceleration"<<endl;
	// cout<<"size of waypoints = "<<waypoints.size()<<endl;
	for(int i = 0; i<waypoints.size() - 1; i++)
	{
		// cout<<"i = "<<i<<endl;
		double current_s = waypoints[i][1][0];
		double current_d = waypoints[i][1][1];
		double next_s = waypoints[i+1][1][0];
		double next_d = waypoints[i+1][1][1];

		double current_timestep_speed_s = (next_s - current_s)/timestep;
		double current_timestep_speed_d = (next_d - current_d)/timestep;
		double current_timestep_speed = vector_sum(current_timestep_speed_s, current_timestep_speed_d);
		speed_waypoints.push_back(current_timestep_speed);
		// cout<<current_timestep_speed<<" ";

		if(i+2 < waypoints.size())
		{
			double next_to_next_s = waypoints[i+2][1][0];
			double next_to_next_d = waypoints[i+2][1][1];
			double next_timestep_speed_s = (next_to_next_s - next_s)/timestep;
			double next_timestep_speed_d = (next_to_next_d - next_d)/timestep;
			double acceleration_s = (next_timestep_speed_s - current_timestep_speed_s)/timestep;
			double acceleration_d = (next_timestep_speed_d - current_timestep_speed_d)/timestep;
			double acceleration = vector_sum(acceleration_s, acceleration_d);
			acceleration_waypoints.push_back(acceleration);
			// cout<<", "<<acceleration<<endl;
		}
	}
	return {speed_waypoints, acceleration_waypoints};
} 

vector<double> speed_cost_trajectories(vector<double> speed_at_every_waypoint, int target_lane_for_state)
{
	double average_speed = 0;
	int waypoints_above_max_speed = 0;
	double max_speed;
	if(target_lane_for_state == 2) max_speed = 40.0;
	else max_speed = MAX_SPEED;

	for(int i = 0; i<speed_at_every_waypoint.size(); i++)
	{
		average_speed += speed_at_every_waypoint[i];
		if(speed_at_every_waypoint[i] >= max_speed) waypoints_above_max_speed++;
	}
	average_speed/= speed_at_every_waypoint.size();

	double average_speed_cost = max(0.0,min(1.0, 1 - (average_speed/max_speed)));
	double above_max_speed_cost = ((double)waypoints_above_max_speed)/((double)speed_at_every_waypoint.size());
	// double above_max_speed_cost = 0.0;
	// if(waypoints_above_max_speed > 0) above_max_speed_cost = 1.0;
	return {average_speed_cost, above_max_speed_cost};
}

vector<double> acceleration_cost_trajectories(vector<double> acceleration_at_every_waypoint)
{
	double average_acceleration = 0;
	int waypoints_above_max_acceleration = 0;

	for(int i = 0; i<acceleration_at_every_waypoint.size(); i++)
	{
		average_acceleration += acceleration_at_every_waypoint[i];
		if(fabs(acceleration_at_every_waypoint[i]) >= MAX_ACCELERATION) waypoints_above_max_acceleration++;
	}
	average_acceleration/= acceleration_at_every_waypoint.size();

	double average_acceleration_cost = min(1.0, 1 - (average_acceleration/MAX_ACCELERATION));//Supports higher acceleration
	double above_max_acceleration_cost = ((double)waypoints_above_max_acceleration)/((double)acceleration_at_every_waypoint.size());//Penalizes trajectories with more waypoints having acceleration higher than MAX_ACCELERATION
	// double above_max_acceleration_cost = 0.0;
	// if(waypoints_above_max_acceleration > 0) above_max_acceleration_cost = 1.0;
	return {average_acceleration_cost, above_max_acceleration_cost};
}

double offset_cost(vector<double> goal_coordinates, int target_lane_for_state)
{
	double lane_center = (4.0 * target_lane_for_state) + 2.0;
	cout<<"In the offset cost() ->"<<endl<<"lane center, goal d"<<endl;
	cout<<lane_center<<", "<<goal_coordinates[1]<<endl;
	double cost = min(1.0, abs(goal_coordinates[1] - lane_center)/4.0);
	return cost;
}

double jerk_cost()
{}

double calculate_cost_within_state(Vehicle &vehicle, int target_lane_for_state, string goal_state, vector<vector<vector<double>>> waypoints)
{
	// cout<<"Printing costs within state:"<<endl;
	vector<vector<double>> speed_acceleration_waypoints_var = speed_acceleration_waypoints(waypoints);
	vector<double> speed_waypoints = speed_acceleration_waypoints_var[0];
	vector<double> acceleration_waypoints = speed_acceleration_waypoints_var[1];
	vector<double> goal_coordinates = waypoints[waypoints.size() - 1][1];
	vector<double> weights = {30,0,70,0,70,40};

	double distance_cost = weights[0] * distance_from_traffic_vehicle_cost(vehicle, target_lane_for_state, goal_state,"TRAJECTORY");
	// cout<<"Distance cost = "<<distance_cost<<endl;
	vector<double> speed_cost = speed_cost_trajectories(speed_waypoints, target_lane_for_state);

	double average_speed_cost = weights[1] * speed_cost[0];
	double over_max_speed_cost = weights[2] * speed_cost[1];
	// cout<<"Average speed cost = "<<average_speed_cost<<endl;
	// cout<<"Over max speed cost = "<<over_max_speed_cost<<endl;

	vector<double> acceleration_cost = acceleration_cost_trajectories(acceleration_waypoints);
	double average_acceleration_cost = weights[3] * acceleration_cost[0];
	double over_max_acceleration_cost = weights[4] * acceleration_cost[1];
	// cout<<"Average acceleration cost = "<<average_acceleration_cost<<endl;
	// cout<<"Over max acceleration cost = "<<over_max_acceleration_cost<<endl;

	double offset_cost_var = weights[5] * offset_cost(goal_coordinates, target_lane_for_state);
	cout<<"Offset cost = "<<offset_cost_var<<endl;
	//ADD JERK COST
	return (distance_cost + average_speed_cost + over_max_speed_cost + average_acceleration_cost + over_max_acceleration_cost + offset_cost_var);
}

#endif
//PROBLEMS

// 1. Velocity and acceleration still crossing the limit even after heavily penalizing the cost functions. - try increasing the number of probable states.
// 2. Think whether the velocity and acceleration of different probable goal states should be different? - IMPLEMENT THIS, BUT HOW?
// 3. The randomly generated 's' sometimes goes beyond the position of the vehicle in front and then calculates the distance assuming it to be behind the vehicle. Resolve this! - decreasing the std_dev_s might help.