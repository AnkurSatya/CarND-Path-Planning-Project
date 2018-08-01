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
	return 1 - (1.0/(1+exp(-x)));
}

//cost for not driving at the maximum possible traffic speed and not driving in the fastest lane.
double inefficiency_cost(Vehicle &vehicle)
{
	if(vehicle.predictions.count(ego_id) == 0) return MAX_COST;//When a successor of a state leads to an infeasible trajectory.

	vector<double> lane_speed(NUM_LANES,-1.0);
	map<int, vector<double>>::iterator it = vehicle.predictions.begin();
	for(it; it != vehicle.predictions.end(); ++it)
	{
		if(it->first == ego_id) continue; //To avoid the scenario where ego vehicle is not moving but still in the fastest available lane.
		int lane = it->second[3]/4;
		if(lane_speed[lane] == -1.0) lane_speed[lane] = it->second[1];
	}

	//If any lane speed is still not defined.
	for(int i = 0; i<lane_speed.size(); i++) if(lane_speed[i] == -1.0) lane_speed[i] = MAX_SPEED;

	int max_speed_lane = max_element(lane_speed.begin(), lane_speed.end()) - lane_speed.begin();
	double max_speed = lane_speed[max_speed_lane];
	double speed_cost = (max_speed - vehicle.predictions.find(ego_id)->second[1])/max_speed;

	// cout<<"fastest lane and speed "<<max_speed_lane<<", "<<max_speed<<endl;
	double lane_cost = (pow(max_speed_lane- int(vehicle.predictions.find(ego_id)->second[3]/4), 2))/pow(NUM_LANES,2);//cost for distance between the max speed and ego's lane.
	return (speed_cost+lane_cost)/2.0;
}

double buffer_distance_cost(Vehicle &vehicle)
{
	map<int, vector<double>>::iterator it = vehicle.predictions.begin();
	int ego_lane = vehicle.predictions.find(ego_id)->second[3]/4;
	double ego_s = vehicle.predictions.find(ego_id)->second[0];
	double min_distance = 1000;

	for(it; it != vehicle.predictions.end(); it++)
	{
		if(it->first == ego_id) continue;
		else
		{
			if(int(it->second[3]/4) == ego_lane && it->second[0] > ego_s) min_distance = min(min_distance, it->second[0] - ego_s);
		}
	}

	double cost = min(1.0,abs(min_distance - BUFFER_DISTANCE)); //The cost = |x-3| bounded between 0 and 1.
	return cost;
}

double max_speed_cost(Vehicle &vehicle)
{
	return (MAX_SPEED - vehicle.predictions.find(ego_id)->second[1])/MAX_SPEED;
}

double max_acceleration_cost(Vehicle &vehicle)
{
	return (MAX_ACCELERATION - vehicle.predictions.find(ego_id)->second[2]);
}

double calculate_cost(Vehicle &vehicle)
{
	return 100*sigmoid(inefficiency_cost(vehicle) + buffer_distance_cost(vehicle) + max_speed_cost(vehicle) +max_acceleration_cost(vehicle));
}

#endif