#include "behavior_planner.h"
#include "behavior_planner_costs.h"
#include <functional>
#include <typeinfo>

using namespace std;

struct CompareSecond
{
	bool operator() (const std::pair<std::string, double> left, std::pair<std::string, double> right) const
	{
		return left.second < right.second;
	}
};

Vehicle::Vehicle(vector<double> ego_state)
{
	// double acc_s = ego_state[2];
	// if(ego_state[1] == 0)
	// {
	// 	acc_s = MAX_ACCELERATION;
	// 	double velocity = min(MAX_ACCELERATION * dt, MAX_SPEED);
	// }
	// ego_state[2] = acc_s;
	vehicles[ego_id] = ego_state;
}

Vehicle::~Vehicle(){}

FSM::FSM() 
{
	// current_state = "KL";//Initial value for state - Keep Lane
	state_graph["KL"] = {"KL","PLCL","PLCR"};
	state_graph["PLCL"] = {"KL","LCL","PLCL"};
	state_graph["LCL"] = {"KL","PLCL"};//Added LCL so that the vehicle moves to the lane center based on the value of the cost function(distance from the lane's center), KL and PLCL would give the same result for this cost function.
	state_graph["PLCR"] = {"KL","LCR","PLCR"};
	state_graph["LCR"] = {"KL","PLCR"};//	
}
FSM::~FSM() {}

void Vehicle::predict()
{
	// cout<<"in predict, vehicle id 0 d value - "<<vehicles.find(0)->second[3]<<endl;
	map<int, vector<double>>::iterator it = vehicles.begin();
	cout<<"IN PREDICT----"<<endl;
	cout<<"VEHICLES SIZE = "<<vehicles.size()<<endl;
	//Leaving the ego vehicle's state unchanged as it will be decided by behavior planner.
	// int count = 0;
	// int test_id;
	for(it; it != vehicles.end(); ++it)
	{
		if(it->first == ego_id) continue;
		// if(!count)
		// {
		// 	count++;
		// 	test_id = it->first;
		// 	// cout<<"in main loop - "<<" test vehicle state size - "
		// }
		vector<double> vehicle_state = it->second;
		vector<double> update = {vehicle_state[1] * dt, 0, 0, vehicle_state[4] * dt, 0, 0};//Although the traffic vehicles are assumed to be moving at a constant speed. I have included this to tinker with the code after the project.
		std::transform(update.begin(), update.end(), vehicle_state.begin(), update.begin(), plus<double>());

		for(int i = 0; i<update.size(); i++)
		{
			if(fabs(update[i]) < 0.0001) update[i] = 0.0;//Approximating the extremely small floating point values to zero.
		}
		update[0] = fmod(update[0], max_s);
		predictions.insert(pair<int, vector<double>> (it->first, update));
	}

	// cout<<"---------------------"<<endl;
	// cout<<"test id "<<test_id<<endl;
	// vector<double>traffic = vehicles.find(test_id)->second;
	// if(traffic.size())
	// {
	// 	cout<<"Traffic vehicle's position -- "<<"initial "<<"final "<<endl;
	// 	cout<<"                              "<<traffic[0]<<"     "<<predictions.find(test_id)->second[0]<<endl;
	// 	cout<<"---------------------"<<endl;
	// }
}

int Vehicle::find_vehicle_ahead(int target_lane)
{
	cout<<"Check3"<<endl;
	int id = ego_id;
	// if(predictions.size() == 0) return id;
	map<int, vector<double>>::iterator it = vehicles.begin();
	++it;
	for(it; it != vehicles.end(); ++it)
	{
		vector<double> new_vehicle_state = vehicles.find(it->first)->second;
		if(int(new_vehicle_state[3]/4) == target_lane) 
		{
			if(vehicles.find(ego_id)->second[0] - new_vehicle_state[0] > 500.0)
			{
				id = it->first;
			}
			else if(new_vehicle_state[0] - vehicles.find(ego_id)->second[0] <= SEARCH_DISTANCE && new_vehicle_state[0] > vehicles.find(ego_id)->second[0]) id = it->first;//checking for the vehicle in the target lane.
		}
	}
	return id;
}

int Vehicle::find_vehicle_behind(int target_lane)
{
	int id = ego_id;
	if(predictions.size() == 0) return id;
	map<int, vector<double>>::iterator it = vehicles.begin();
	++it;

	for(it; it != vehicles.end(); ++it)
	{
		vector<double> new_vehicle_state = vehicles.find(it->first)->second;
		if(int(new_vehicle_state[3]/4) == target_lane) 
		{
			if (new_vehicle_state[0] - vehicles.find(ego_id)->second[0] > 500.0) it->first;
			else if(vehicles.find(ego_id)->second[0] - new_vehicle_state[0] <= SEARCH_DISTANCE && new_vehicle_state[0] < vehicles.find(ego_id)->second[0]) id = it->first;//checking for the vehicle in the target lane.
		}
	}
	return id;
}

vector<double> Vehicle::get_kinematics(Vehicle &vehicle, int target_lane, string goal_state)
{
	int vehicle_ahead_id = find_vehicle_ahead(target_lane);
	int vehicle_behind_id = find_vehicle_behind(target_lane);

	// vector<double> ahead_vehicle = vehicle.vehicles.find(vehicle_ahead_id)->second;
	// vector<double> behind_vehicle = vehicle.vehicles.find(vehicle_behind_id)->second;
	// if(vehicle_ahead_id != ego_id) 
	// {
	// 	cout<<"Vehicle Ahead ID, Lane, Delta s "<<vehicle_ahead_id<<'\t'<<int(ahead_vehicle[3]/4.0)<<'\t'<<ahead_vehicle[0] - vehicle.vehicles.find(ego_id)->second[0]<<endl;	
	// }
	// if(vehicle_behind_id != ego_id) 
	// {
	// 	cout<<"Vehicle Behind ID, Delta s "<<vehicle_behind_id<<'\t'<<int(behind_vehicle[3]/4.0)<<'\t'<<vehicle.vehicles.find(ego_id)->second[0] - behind_vehicle[0]<<endl;
	// }
	vector<double> new_kinematics;
	// cout<<"Check2"<<endl;
	vector<double> current_kinematics = vehicle.vehicles.find(ego_id)->second;
	// cout<<"Check 3"<<endl;
	double s = current_kinematics[0];
	double s_dot = current_kinematics[1];
	double s_dot_dot = current_kinematics[2];
	double d = current_kinematics[3];
	double d_dot = current_kinematics[4];
	double d_dot_dot = current_kinematics[5];
	double new_velocity;	
	cout<<"INSIDE GET KINEMATICS----------"<<endl;
	cout<<"Target Lane = "<<target_lane<<endl;
	cout<<"TRAFFIC VEHICLE ID = "<< vehicle_ahead_id<<endl;

	double max_speed = MAX_SPEED;
	// if(goal_state.compare("KL") && target_lane == 2) max_speed-=5;//Done because vehicles goes beyond the maximum velocity when in the final lane.
	//If there is traffic, move with the traffic's speed.
	if(vehicle_ahead_id != -1)
	{
		vector<double> vehicle_ahead_kinematics = vehicle.vehicles.find(vehicle_ahead_id)->second;
		if(vehicle_behind_id != -1) 
		{
			new_velocity = min(vehicle_ahead_kinematics[1], max_speed);
			cout<<"Ahead and Behind"<<endl;
		}
		else
		{
			double distance_between = vehicle_ahead_kinematics[0] - s;
			if(vehicle_ahead_kinematics[0] < s) distance_between = (max_s - s + vehicle_ahead_kinematics[0]);

			new_velocity = (distance_between - BUFFER_DISTANCE -0.5*s_dot_dot*dt*dt)/dt + vehicle_ahead_kinematics[1];// The acceleration term is added to negate the effect of the acceleration at the previous timestamp on the new velocity.
			new_velocity = min(min(MAX_ACCELERATION*dt + s_dot, new_velocity), max_speed);
			cout<<"Vehicle Ahead S = "<<vehicle_ahead_kinematics[0]<<endl;
			cout<<"Ego Vehicle S = "<<s<<endl;
			// cout<<"distance between = "<<distance_between<<endl;
			cout<<"acceleration = "<<s_dot_dot<<endl;
			cout<<"Velocity as evaluated inside get_kinematics = "<<new_velocity<<endl;
			cout<<"Only Ahead"<<endl;
		}
	}
	else//keeps a check on both max acceleration and max speed.
	{	
		new_velocity = min(MAX_ACCELERATION * dt + s_dot, max_speed);
		cout<<"No vehicle nearby"<<endl;
	}
	
	if(new_velocity < 0) 
	{
		new_velocity = 0.0;
		s_dot_dot = 0.0;
	}
	else s_dot_dot = min(MAX_ACCELERATION,(new_velocity - s_dot)/dt);
	// s += new_velocity * dt + 0.5 * s_dot_dot * dt * dt;
	s+= s_dot*dt + 0.5*s_dot_dot*dt*dt;
	// d_dot_dot = 0.0;
	// d_dot = 0.0;
	// cout<<"Printing from get kinematics ->"<<endl;
	// cout<<"S = "<<s<<endl;
	cout<<"New predicted velocity of ego vehicle = "<<new_velocity<<endl;
	// cout<<"S_dot_dot = "<<s_dot_dot<<endl;
	// d+= d_dot * dt + 0.5 * s_dot_dot * dt *dt;//D update is not necessary as behavior planner is used to predict the end states not the trajctory which means the vehicle will always be heading towards the lane direction. 
	new_kinematics = {s, new_velocity, s_dot_dot, 4.0*target_lane + 2.0, d_dot, d_dot_dot}; //converting d to integer to avoid the final position of the ego vehicle from being outside a lane.
	return new_kinematics;
}

int FSM::find_goal_pose(Vehicle &vehicle, string start_state, string goal_state, int target_lane)
{
	// int target_lane = (vehicle.vehicles.find(ego_id)->second[3])/4 + stoi_state.find(goal_state)->second;
	// cout<<"target_lane "<<target_lane<<endl;

	// if(target_lane < d_min || target_lane > d_max) return 1;
	cout<<"Check1"<<endl;
	vector<double> new_lane_kinematics = vehicle.get_kinematics(vehicle, target_lane, goal_state);
	cout<<"Check2"<<endl;

	if(goal_state.compare("KL") == 0)//KEEP LANE
	{
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id, new_lane_kinematics));
	}

	else if(goal_state.compare("PLCL") == 0|| goal_state.compare("PLCR") == 0)//PREPARE LANE CHANGE LEFT
	{
		vector<double> current_lane_new_kinematics = vehicle.get_kinematics(vehicle, vehicle.vehicles.find(ego_id)->second[3]/4, goal_state);
		vector<double> best_kinematics = current_lane_new_kinematics;

		if(vehicle.find_vehicle_behind(target_lane) == -1)
		{
			if(current_lane_new_kinematics[1] > new_lane_kinematics[1]) 
			{
				
				best_kinematics = new_lane_kinematics;
			}
			best_kinematics[3] = current_lane_new_kinematics[3];//keeping the lane after the prediction same as the initial lane.
		}	
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id, best_kinematics));	
	}

	else if(goal_state.compare("LCL")== 0 ||  goal_state.compare("LCR") == 0)//Change lane.
	{
		map<int,vector<double>>::iterator it = vehicle.predictions.begin();
		for(it; it != vehicle.predictions.end(); ++it)
		{
			if(it->first == ego_id) continue;
			// If another vehicle is already at the predicted position of the ego vehicle, then abort the FSM state change.
			if(int(vehicle.predictions.find(it->first)->second[3]/4.0) == target_lane)
			{
				double traffic_predicted_s = vehicle.predictions.find(it->first)->second[0];
				double distance_between = fabs(traffic_predicted_s - new_lane_kinematics[0]);

				if(distance_between > 200.0) distance_between = fabs(fmod(max(max_s - traffic_predicted_s, new_lane_kinematics[0]), max_s) + min(max_s - traffic_predicted_s, new_lane_kinematics[0]));//To be used when car completes a rouund.
				// cout<<"Distance between = "<<distance_between<<endl;
				if(distance_between <= BUFFER_DISTANCE)
				{
					cout<<"Vehicle at predicted location."<<endl;
					// cout<<"Vehicle Info:"<<endl<<"    "<<"Vehicle ID = "<<it->first<<endl<<"    "<<"Vehicle Lane = "<<int(vehicle.predictions.find(it->first)->second[3]/4.0)<<endl;
					cout<<"    "<<"Distance from ego vehicle = "<<distance_between<<endl;
					return 2;
				}
			}
		}		
		// new_lane_kinematics[0]+=3.0;
		new_lane_kinematics[3] = double(int((vehicle.vehicles.find(ego_id)->second[3] + stoi_state.find(goal_state)->second * 4.0)/4)*4) + 2.0;
		vehicle.predictions.insert(pair<int, vector<double>>(ego_id,new_lane_kinematics));
	}
	return 0;
}

vector<vector<double>> FSM::next_state(Vehicle &vehicle, string &current_state, int &current_lane)
{
	vector<string> successors = state_graph.find(current_state)->second;
	map<string, double>costs;//Map for storing costs for all the trajectories.
	double min_cost = 1000*MAX_COST;
	string min_cost_state;
	vector<double> min_cost_trajectory;
	vector<vector<double>> trajectory;
	trajectory.push_back(vehicle.vehicles.find(ego_id)->second);

	vehicle.predict();
	// cout<<"In the next state function:"<<endl;
	// cout<<"------------------"<<endl;
	// cout<<"Current Vehicle position(S) = "<<vehicle.vehicles.find(ego_id)->second[0]<<endl;
	cout<<"Current State - "<<current_state<<endl;
	cout<<"Current Lane - "<<current_lane<<endl;
	
	for(int i = 0; i<successors.size(); i++)
	{	
		string goal_state = successors[i];
		cout<<"Successor - "<<goal_state<<endl;

		// int target_lane_for_state = int(vehicle.vehicles.find(ego_id)->second[3])/4 + stoi_state.find(goal_state)->second;

		int target_lane_for_state = current_lane + stoi_state.find(goal_state)->second;
		cout<<"target_lane_for_state - "<<target_lane_for_state<<endl;
		
		if(target_lane_for_state < d_min || target_lane_for_state > d_max)
		{
			costs[goal_state] = MAX_COST;
			cout<<"Total Cost = "<<MAX_COST<<endl;
			continue;
		}

		int ret = find_goal_pose(vehicle, current_state, goal_state, target_lane_for_state);
		if(ret == 1 || ret == 2)
		{
			costs[goal_state] = MAX_COST;
			cout<<"Total Cost = "<<MAX_COST<<endl;
			continue;
		}

		cout<<"Predicted Vehicle Position(S) = "<<vehicle.predictions.find(ego_id)->second[0]<<endl;

		// cout<<"Ego vehicle's goal state velocity = "<<vehicle.predictions.find(ego_id)->second[1]<<endl;
		double total_cost = calculate_cost(vehicle, target_lane_for_state, goal_state);

		cout<<"Total cost = "<<total_cost<<endl;

		if(total_cost < min_cost) 
		{
			min_cost_state = goal_state;
			min_cost = total_cost;
			min_cost_trajectory = vehicle.predictions.find(ego_id)->second;
		}
		costs[goal_state] = total_cost;
		vehicle.predictions.erase(ego_id);
	}

	//Predicting future state for traffic vehicles.
	// vehicle.predict();

	//Choosing the minimum cost state.
	// std::pair<std::string, double> min_cost_element = *min_element(costs.begin(), costs.end(), CompareSecond());
	// min_cost_state = min_cost_element.first;
	// min_cost = min_cost_element.second;
	// cout<<"Minimum Cost = "<<min_cost<<endl;

	// cout<<"Final State - "<<min_cost_state<<endl;
	// if(current_state.compare(min_cost_state))//When they do not match.
	// {
		// cout<<"---------States Changed-----------"<<endl;
		// cout<<"current_state - "<<current_state<<endl;
		// map<string,double>::iterator it = costs.begin();
		// for(it; it != costs.end(); ++it)
		// {
		// 	cout<<"State name, total cost - "<<it->first<<", "<<it->second<<endl;
		// }
		cout<<"Final state chosen - "<<min_cost_state<<endl;
		// cout<<"------------------------"<<endl;
	// }
	if(costs.size() == 0) cout<<"All successors were evaluated to be infeasible. There is something wrong with the code."<<endl;
	current_state = min_cost_state;
	current_lane = min_cost_trajectory[3]/4.0;

	trajectory.push_back(min_cost_trajectory);
	// cout<<"initial velocity "<<trajectory[0][1]<<" final velocity "<<trajectory[1][1]<<endl;
	cout<<"initial s = "<<trajectory[0][0]<<", final s = "<<trajectory[1][0]<<endl;
	cout<<"initial d = "<<trajectory[0][3]<<", final d = "<<trajectory[1][3]<<endl;
	return trajectory;
}

vector<vector<vector<double>>> FSM::generate_multiple_goal_states(vector<vector<double>> original_state, int num_states, double std_dev_s, double std_dev_d)
{
	vector<vector<vector<double>>> goal_states;
	vector<double> original_goal_state = original_state[1];

	int num_goal_states_ahead = num_states/2;
	int i = 0;
	// cout<<"Mean S = "<<original_goal_state[0]<<endl;
	// cout<<"Printing probable goal states:"<<endl;
	// cout<<"S, D->"<<endl;

	while(i<num_states)
	{
		vector<vector<double>> new_state;
		new_state.push_back(original_state[0]);

		vector<double> goal_state = original_goal_state;
		double s;
		double d;
		// double d_min = int(original_goal_state[3]/4)
		if(i%2 == 0) 
		{
			s = original_goal_state[0] + ((double)rand()/RAND_MAX) * std_dev_s;
			d = original_goal_state[3] + ((double)rand()/RAND_MAX) * std_dev_d;
		}
		else
		{
			s = original_goal_state[0] - ((double)rand()/RAND_MAX) * std_dev_s;
			d = original_goal_state[3] - ((double)rand()/RAND_MAX) * std_dev_d;
			// d = min()
		}
		
		// cout<<"D VALUE = "<<d<<endl;
		goal_state[0] = s;
		goal_state[3] = d;
		new_state.push_back(goal_state);
		goal_states.push_back(new_state);
		i++;
	}
	return goal_states;
}

vector<vector<vector<double>>> FSM::best_state_waypoints(vector<vector<double>> original_goal_state,Vehicle &vehicle, generate_trajectory traj, vector<vector<vector<double>>> goal_states, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, int num_waypoints, int num_waypoints_from_previous_path, string goal_state_name)
{
	// cout<<"Selecting the best trajectory for the chosen state...."<<endl;
	vehicle.predictions.insert(std::pair<int, vector<double>>(ego_id,goal_states[0][1]));
	int target_lane_for_state = goal_states[0][1][3]/4;
	double min_cost = MAX_COST;
	vector<vector<vector<double>>> best_trajectory;
	vector<vector<double>> costs;
	double ego_predicted_s = original_goal_state[1][0];

	for(int i = 0; i<goal_states.size(); i++)
	{
		vector<double> cost;
		vector<vector<vector<double>>> jmt_waypoints;
		vector<vector<double>> goal_state = goal_states[i];
		
		vector<vector<double>> goal_state_for_coeffs = goal_state;//this modifies the 'S' of the goal state by a parameter to create a smooth trajectory.
		goal_state_for_coeffs[1][0]+=20.0;
		// goal_state[0] = source_state;
		// cout<<"DEBUGGING---"<<endl;
		// for(int i = 0; i<goal_state[0].size(); i++)
		// {
		// 	cout<<source_state[i]<<", "<<goal_state[1][i]<<endl;
		// }
		vector<vector<double>> coeffs = traj.generate_JMT(goal_state_for_coeffs);

		int jmt_waypoints_size = 1.0*num_waypoints;
		if(num_waypoints_from_previous_path == 0) jmt_waypoints_size = 1.0*num_waypoints;

		for(double j = 1; j<=jmt_waypoints_size; j++)
        {
	        double t = j*timestep;
      		vector<vector<double>> jmt_waypoint = traj.generate_JMT_waypoints(coeffs, t, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	        jmt_waypoints.push_back(jmt_waypoint);
        }

        double distance_between = goal_state[1][0] - ego_predicted_s;
        std::map<int, vector<double>>::iterator it = vehicle.predictions.find(ego_id);
        it->second[0] = goal_state[1][0];
        it->second[3] = goal_state[1][3]; 
    	double total_cost = calculate_cost_within_state(vehicle,target_lane_for_state, goal_state_name, jmt_waypoints);

    	cost.push_back(fabs(distance_between)/distance_between);
    	cost.push_back(total_cost);
    	costs.push_back(cost);

    	if(total_cost < min_cost)
    	{
    		best_trajectory = jmt_waypoints;
	    	min_cost = total_cost;  		
    	}
	}

	// if(goal_state_name.compare("KL") || goal_state_name.compare("PLCL") || goal_state_name.compare("PLCR"))
	// if(goal_state_name.compare("LCL") == 0 || goal_state_name.compare("LCR") == 0)
	// {
	// 	cout<<"GOAL STATE NAME = "<<goal_state_name<<endl;
		// cout<<"PRINTING THE NEWLY GENERATED WAYPOINTS"<<endl;
		// for(int i = 0; i<best_trajectory.size(); i++)
		// {
		// 	cout<<best_trajectory[i][1][0]<<", "<<best_trajectory[i][1][1]<<endl;
		// }
	// }
	// cout<<"Costs for all the goal states->"<<endl;
	// for(int i =0; i<costs.size(); i++)
	// {
	// 	cout<<costs[i][0]<<", "<<costs[i][1]<<endl;
	// }

	cout<<"Final trajectory chosen cost = "<<min_cost<<endl;
	return best_trajectory;
}
