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
		if(it->first == ego_id) continue;
		vector<double> traffic_vehicle_state = it->second;

		int lane_id = traffic_vehicle_state[3]/4;
		
		if(lane_id <d_min || lane_id > d_max)
		{
			if(lane_id < d_min) lane_id++;
			else lane_id --;
		}

		if(traffic_vehicle_state[3] < -1.0 || traffic_vehicle_state[3] > 13.5) continue;
		double delta_s = traffic_vehicle_state[0] - ego_vehicle_state[0];
		// cout<<"Vehicle ID, D value = "<<it->first<<", "<<traffic_vehicle_state[3]<<endl;
		if(delta_s > 0.0 && delta_s < lane_distance_speed[lane_id][1] && delta_s <= 70.0)//Used 70.0(a high distance value) to check the fastest lane for better lane change decision making.
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

	// if(target_lane_for_state == int(ego_vehicle_state[3]/4))
	// {
	// 	if(max_speed - lane_distance_speed[target_lane_for_state][2]<1.0)
	// 	{
	// 		max_speed = lane_distance_speed[target_lane_for_state][2];
	// 	}
	// }
	// vector<vector<double>> lane_distance_speed_copy = lane_distance_speed;
	// std::sort(lane_distance_speed_copy.begin(), lane_distance_speed_copy.end(), sort_by_speed);
	
	// if(lane_distance_speed_copy[0][0] == target_lane_for_state)
	// vector<int> lane_distance_speed_copy_index;
	// lane_distance_speed_copy_index.push_back(0);

	// for(int i = 0; i<NUM_LANES-1;i++)
	// {
	// 	if(lane_distance_speed_copy[i][2] - lane_distance_speed_copy[i+1][2] <= 1.0) lane_distance_speed_copy_index.push_back(i+1);
	// }

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
	// if(max_speed - lane_distance_speed[target_lane_for_state][2] <= 0.5) speed_cost = 0.0;// If difference in speed is less than 0.5 m/s, then inefficiency cost is 0.
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
		if(goal_state.compare("PLCL") == 0 || goal_state.compare("PLCR") == 0) ego_predicted_lane = vehicle.predictions.find(ego_id)->second[3]/4;
	}

	double ego_predicted_s = vehicle.predictions.find(ego_id)->second[0];
	double min_distance_ahead = 1000;
	double min_distance_behind = 1000;
	int ahead_id = ego_id;
	int behind_id = ego_id;
	double check_distance = SEARCH_DISTANCE;

	for(it; it != vehicle.predictions.end(); it++)
	{
		if(it->first == ego_id) continue;
		else
		{
			if(int(it->second[3]/4) == ego_predicted_lane && fabs(it->second[0] - ego_predicted_s) <= check_distance) 
			{
				cout<<"Checking distance from the chosen traffic vehicle = "<<it->second[0] - ego_predicted_s<<endl;
				cout<<"Vehicle Lane = "<<ego_predicted_lane<<endl;
				cout<<"Velocity of the traffic vehicle = "<<it->second[1]<<endl<<endl;
				if(it->second[0] > ego_predicted_s) 
				{
					if(min_distance_ahead > (it->second[0] - ego_predicted_s))
					{
						min_distance_ahead = it->second[0] - ego_predicted_s;
						ahead_id = it->first;						
					}
				}
				else if(it->second[0] < ego_predicted_s)
				{
					if(min_distance_behind > (ego_predicted_s - it->second[0]))
					{
						min_distance_behind = ego_predicted_s - it->second[0];
						behind_id = it->first;
					}
				}
			}
		}
	}

	double buffer_distance_ahead = BUFFER_DISTANCE;
	double buffer_distance_behind = BUFFER_DISTANCE;
	double cost_ahead = max(0.0, 1.0 - (min_distance_ahead/check_distance));
	double cost_behind = max(0.0, 1.0 - (min_distance_behind/check_distance));

	// if(min_distance_ahead >= buffer_distance_ahead) cost_ahead = exp(-SEARCH_DISTANCE/min_distance_ahead);
	// else cost_ahead = exp(-min_distance_ahead);

	// if(min_distance_behind>= buffer_distance_behind) cost_behind = exp(-SEARCH_DISTANCE/min_distance_behind);
	// else cost_behind = exp(-min_distance_behind);

	// double cost_ahead = 1.0 - (1/(1+exp(-(min_distance_ahead - max(buffer_distance_ahead-4.5,4.5)))));//subtracted 4 from buffer distance because sigmoid peaks at x=4 and we want to shift the graph right by (buffer distance - 4).
	// double cost_behind = 1.0 - (1/(1+exp(-(min_distance_behind - max(buffer_distance_behind-4.5,4.5)))));
	double cost = (cost_ahead + cost_behind);
	return cost;
}

double speed_cost(Vehicle &vehicle)
{
	if(vehicle.predictions.find(ego_id)->second[1] > MAX_SPEED) return MAX_COST;
	double cost = sigmoid((MAX_SPEED - vehicle.predictions.find(ego_id)->second[1])/MAX_SPEED);
	// cout<<"Ego vehicle speed = "<<vehicle.predictions.find(ego_id)->second[1]<<endl;
	cout<<"Speed cost = "<<cost<<endl;
	return cost;
}

double acceleration_cost(Vehicle &vehicle)
{
	if(vehicle.predictions.find(ego_id)->second[2] > MAX_ACCELERATION) return MAX_COST;
	double cost = sigmoid((MAX_ACCELERATION - vehicle.predictions.find(ego_id)->second[2])/MAX_ACCELERATION); 
	// cout<<"Ego vehicle acceleration = "<<vehicle.predictions.find(ego_id)->second[2]<<endl;
	cout<<"Acceleration cost = "<<cost<<endl;
	return cost;
}

double calculate_cost(Vehicle &vehicle, int target_lane_for_state, string goal_state)
{
	if(vehicle.predictions.count(ego_id) == 0) return MAX_COST;
	vector<double> weights = {5,100,0,0};

	cout<<endl;
	cout<<"Individual Costs - "<<endl;
	double ineff_cost = inefficiency_cost(vehicle, target_lane_for_state);
	double distance_cost = distance_from_traffic_vehicle_cost(vehicle, target_lane_for_state, goal_state, "STATE");
	double sp_cost = speed_cost(vehicle);
	double acc_cost = acceleration_cost(vehicle);
	cout<<"Distance cost = "<<distance_cost<<endl;
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

vector<vector<double>> speed_acceleration_jerk_waypoints(vector<vector<vector<double>>> waypoints)
{
	vector<double> speed_waypoints;
	vector<double> acceleration_waypoints;
	vector<double> jerk_waypoints;
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

		double acceleration_s;
		double acceleration_d;
		if(i+2 < waypoints.size())
		{
			double next_to_next_s = waypoints[i+2][1][0];
			double next_to_next_d = waypoints[i+2][1][1];
			double next_timestep_speed_s = (next_to_next_s - next_s)/timestep;
			double next_timestep_speed_d = (next_to_next_d - next_d)/timestep;
			acceleration_s = (next_timestep_speed_s - current_timestep_speed_s)/timestep;
			acceleration_d = (next_timestep_speed_d - current_timestep_speed_d)/timestep;
			double acceleration = vector_sum(acceleration_s, acceleration_d);
			acceleration_waypoints.push_back(acceleration);
			// cout<<", "<<acceleration<<endl;
		}
		if(i+3 < waypoints.size())
		{
			double s_2 = waypoints[i+1][1][0];
			double d_2 = waypoints[i+1][1][1];
			double s_3 = waypoints[i+2][1][0];
			double d_3 = waypoints[i+2][1][1];
			double s_4 = waypoints[i+3][1][0];
			double d_4 = waypoints[i+3][1][1];

			double speed_2_3_s = (s_3 - s_2)/timestep;
			double speed_2_3_d = (d_3 - d_2)/timestep;
			double speed_3_4_s = (s_4 - s_3)/timestep;
			double speed_3_4_d = (d_4- d_3)/timestep;

			double acceleration_2_4_s = (speed_3_4_s - speed_2_3_s)/timestep;
			double acceleration_2_4_d = (speed_3_4_d - speed_2_3_d)/timestep;

			double jerk_s = (acceleration_2_4_s - acceleration_s);
			double jerk_d = (acceleration_2_4_d - acceleration_d);
			double jerk = vector_sum(jerk_s, jerk_d);
			jerk_waypoints.push_back(jerk);
		}
	}
	return {speed_waypoints, acceleration_waypoints, jerk_waypoints};
} 

vector<double> speed_cost_trajectories(vector<double> speed_at_every_waypoint, int target_lane_for_state)
{
	double average_speed = 0;
	int waypoints_above_max_speed = 0;
	double max_speed = MAX_SPEED;

	double reduction_max_speed = 0.0;//1.0
	if(target_lane_for_state == 2) reduction_max_speed = 0.0;//1.5

	for(int i = 0; i<speed_at_every_waypoint.size(); i++)
	{
		average_speed += speed_at_every_waypoint[i];
		if(speed_at_every_waypoint[i] >= max_speed-reduction_max_speed) waypoints_above_max_speed++;
	}
	average_speed/= speed_at_every_waypoint.size();
	// cout<<"WAYPOINTS above max speed = "<<waypoints_above_max_speed<<endl;	

	double average_speed_cost = max(0.0,min(1.0, 1 - (average_speed/max_speed)));
	// double average_speed_cost = 0.0;
	if(average_speed>MAX_SPEED - 1.5) average_speed_cost = min(1.0, average_speed/max_speed);
	
	double above_max_speed_cost = ((double)waypoints_above_max_speed)/((double)speed_at_every_waypoint.size());
	// double above_max_speed_cost = 1/(1+exp(-(max_speed - average_speed)));
	// double above_max_speed_cost = average_speed/max_speed;
	// if(target_lane_for_state == 2) above_max_speed_cost*=4;
	// double above_max_speed_cost = 0.0;
	// if(waypoints_above_max_speed > 0) above_max_speed_cost = 1.0;
	return {average_speed_cost, above_max_speed_cost};
}

vector<double> acceleration_cost_trajectories(vector<double> acceleration_at_every_waypoint, int target_lane_for_state)
{
	double average_acceleration = 0;
	int waypoints_above_max_acceleration = 0;
	double max_acceleration = MAX_ACCELERATION;
	double reduction_max_acceleration = 2.0;
	if(target_lane_for_state == 2) reduction_max_acceleration = 2.0;//1.0

	for(int i = 0; i<acceleration_at_every_waypoint.size(); i++)
	{
		average_acceleration += abs(acceleration_at_every_waypoint[i]);
		if(fabs(acceleration_at_every_waypoint[i]) >= max_acceleration - reduction_max_acceleration) waypoints_above_max_acceleration++;
	}
	average_acceleration/= acceleration_at_every_waypoint.size();

	// double average_acceleration_cost = min(1.0, 1 - (average_acceleration/max_acceleration));//Supports higher acceleration
	double average_acceleration_cost = 0.0;
	average_acceleration_cost = min(1.0,average_acceleration/(MAX_ACCELERATION));
	// if(average_acceleration > MAX_ACCELERATION - 3.0) average_acceleration_cost = min(1.0, average_acceleration/(MAX_ACCELERATION - 3.0));

	double above_max_acceleration_cost = ((double)waypoints_above_max_acceleration)/((double)acceleration_at_every_waypoint.size());//Penalizes trajectories with more waypoints having acceleration higher than MAX_ACCELERATION
	// double above_max_acceleration_cost = average_acceleration/MAX_ACCELERATION;
	// if(target_lane_for_state == 2) above_max_acceleration_cost*=4;
	return {average_acceleration_cost, above_max_acceleration_cost};
}

vector<double> jerk_cost_trajectories(vector<double> jerk_at_every_waypoint, int target_lane_for_state)
{
	double average_jerk = 0.0;
	int waypoints_above_max_jerk = 0;
	double max_jerk = MAX_JERK;

	for(int i=0; i<jerk_at_every_waypoint.size(); i++)
	{
		average_jerk+= abs(jerk_at_every_waypoint[i]);
		if(fabs(jerk_at_every_waypoint[i] >= max_jerk)) waypoints_above_max_jerk++;
	}
	average_jerk/=jerk_at_every_waypoint.size();

	double average_jerk_cost = max(1.0, average_jerk/MAX_JERK);
	double max_jerk_cost = double(waypoints_above_max_jerk)/double(jerk_at_every_waypoint.size());

	return {average_jerk_cost, max_jerk_cost};
}

double offset_cost(vector<double> goal_coordinates, int target_lane_for_state, vector<vector<vector<double>>> waypoints)
{
	double lane_center = (4.0 * target_lane_for_state) + 2.0;
	// cout<<"In the offset cost() ->"<<endl<<"lane center, goal d"<<endl;
	// cout<<lane_center<<", "<<goal_coordinates[1]<<endl;
	double cost = 0.0;
	for (int i=0; i<waypoints.size(); i++)
	{
		cost+= min(1.0, abs(waypoints[i][1][1] - lane_center)/4.0);
	}
	cost/=waypoints.size();
	// double cost = min(1.0, abs(goal_coordinates[1] - lane_center)/4.0);
	return cost;
}

double calculate_cost_within_state(Vehicle &vehicle, int target_lane_for_state, string goal_state, vector<vector<vector<double>>> waypoints)
{
	// cout<<endl;
	// vector<vector<double>> speed_acceleration_jerk_waypoints_var = speed_acceleration_jerk_waypoints(waypoints);
	// vector<double> speed_waypoints = speed_acceleration_jerk_waypoints_var[0];
	// vector<double> acceleration_waypoints = speed_acceleration_jerk_waypoints_var[1];
	// vector<double> jerk_waypoints = speed_acceleration_jerk_waypoints_var[2];

	// vector<double> goal_coordinates = waypoints[waypoints.size() - 1][1];
	// vector<double> weights = {0,0,0,0,0,0,0,0};//100,0,80,0,80,60 at max-speed = 45 {20,10,200,10,140,100} at speed = 49.5//{500,50,0,50,0,0,0,5000}

	// // cout<<"C2"<<endl;
	// double distance_cost = weights[0] * distance_from_traffic_vehicle_cost(vehicle, target_lane_for_state, goal_state,"TRAJECTORY");
	// // cout<<"Distance cost = "<<distance_cost<<endl;
	// // cout<<"C3"<<endl;
	// vector<double> speed_cost = speed_cost_trajectories(speed_waypoints, target_lane_for_state);

	// double average_speed_cost = weights[1] * speed_cost[0];
	// double over_max_speed_cost = weights[2] * speed_cost[1];
	// // cout<<"Average speed cost = "<<average_speed_cost<<endl;
	// // cout<<"Over max speed cost = "<<over_max_speed_cost<<endl;

	// // cout<<"C4"<<endl;
	// vector<double> acceleration_cost = acceleration_cost_trajectories(acceleration_waypoints, target_lane_for_state);
	// double average_acceleration_cost = weights[3] * acceleration_cost[0];
	// double over_max_acceleration_cost = weights[4] * acceleration_cost[1];
	// // cout<<"Average acceleration cost = "<<average_acceleration_cost<<endl;
	// // cout<<"Over max acceleration cost = "<<over_max_acceleration_cost<<endl;
	// // cout<<"C5"<<endl;
	// vector<double> jerk_cost = jerk_cost_trajectories(jerk_waypoints, target_lane_for_state);
	// double average_jerk_cost = weights[5] * jerk_cost[0];
	// double over_max_jerk_cost = weights[6] * jerk_cost[1];

	// // cout<<"C6"<<endl;
	// double offset_cost_var = weights[7] * offset_cost(goal_coordinates, target_lane_for_state, waypoints);
	// cout<<"Offset cost = "<<offset_cost_var<<endl;
	// return (distance_cost + average_speed_cost + over_max_speed_cost + average_acceleration_cost + over_max_acceleration_cost + average_jerk_cost + over_max_jerk_cost + offset_cost_var);
	return 0;
}

#endif
//PROBLEMS
//RESOLVED - 1. Vehicle getting very slow while changing lane.(Jittering) - either include LCL for LCL or include more new points in waypoints vector. - not because of high offset cost.
//RESOLVED - 2. Sways in the lane. 
// 3. Need to tweak inefficiency cost function or it could be that distance cost function's weight is too high that is why car changes lane too rapidly. Reduce the difference between the two.
// 4. Might need to include the lane_change_distance parameter to avoid collision while changing lane.

//SOLUTION
//1. Revert the dynamic buffer and search distance changes. DONE
//2. Evaluate every cost function one by one.
// 3. Distance evaluation cost function not appropriate. Fines too much even when the ego vehicle is very far. Also increase the weight as distance cost is very less compared to other costs.
// 4. Acceleration cost too high.
// 5. Sometimes vehicle plans to go in a lane where a vehicle is already present but the new waypoints added prevents it from going there. - Increase weight for the distance cost function.


//Removed PLCL and PLCR from Finite States Machine.
// 1. Not a good idea, vehicle emaluates the behaviour of a bad driver who changes lane rapidly.
// 2. Jittering in lane change still unresolved.

// IF PROBLEM
// 1. Takes turn when a vehicle in the intended lane is near - set a parameter lane_change_distance which will be checked before changing lane. Use a distance cost function which fines the distance from the nearest vehicle irrespective of the lane.

//Changes made at 4:23 on 13/10
//1. To generate more waypoints using JMT, changed value of parameter to the constructor generate_trajectory in the main file. - reverted.
//2. Changing the source state given to the trajectory generatr from last state of the previous path to vehicle's current state.