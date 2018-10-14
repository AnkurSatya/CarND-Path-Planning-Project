#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "json.hpp"
#include <ctime>
#include <chrono>
#include <thread> //For time.sleep()
#include "constants.h"
#include "helper.h"
#include "behavior_planner.h"
#include "trajectory_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  //This is a very poorly designed mathematical formulation. The approximation is not fixed and increases with the curvature, map waypoint and vehicle position. Problem can be solved by using splines to generate more waypoints.
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

int main() {
  uWS::Hub h;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;

  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  vector<vector<double>> upsampled_map_waypoints_x_y_s = upsample_map_waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s);
  vector<double> map_waypoints_x_upsampled = upsampled_map_waypoints_x_y_s[0];
  vector<double> map_waypoints_y_upsampled = upsampled_map_waypoints_x_y_s[1];
  vector<double> map_waypoints_s_upsampled = upsampled_map_waypoints_x_y_s[2];

  map_waypoints_x.clear();
  map_waypoints_y.clear();
  map_waypoints_s.clear();

  map_waypoints_x = map_waypoints_x_upsampled;
  map_waypoints_y = map_waypoints_y_upsampled;
  map_waypoints_s = map_waypoints_s_upsampled;

  vector<double> prev_s;
  vector<double> prev_d;
  int path_size;
  string current_state = "KL";
  int current_lane = 1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &current_state, &current_lane, &prev_s, &prev_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
  
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      // auto start = chrono::high_resolution_clock::now();
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {      
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
            cout<<"Car S from simulator = "<<car_s<<endl;
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];//in degrees.
          	double car_speed = j[1]["speed"];
            car_speed = (car_speed * 1.60934 * 5.0/18.0);//converting MPH to m/s.
            // car_d = 10;
            //Insert car's state as id, s, s_dot, d, d_dot.

            // vector<double> velocity_frenet= getF_velocity(car_x, car_y, car_yaw, car_speed*cos(car_yaw), car_speed*sin(car_yaw), map_waypoints_x, map_waypoints_y);
            // vector<double> source_state(6,0);
            // source_state[0] = car_s;
            // source_state[1] = car_speed;
            // source_state[2] = 0.0;
            // source_state[3] = car_d;
            // source_state[4] = 0.0;
            // source_state[5] = 0.0;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
            // cout<<"Previous path size = "<<previous_path_x.size()<<endl;
            // if(previous_path_x.size() != 0) cout<<"Previous path first waypoint = "<<previous_path_x[0]<<", "<<previous_path_y[0]<<endl;
            // cout<<"Current x and y of car = "<<car_x<<", "<<car_y<<endl;
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];//returns a vector of strings
            cout<<"SENSOR FUSION SIZE ="<<sensor_fusion.size()<<endl; 
            // for(int i = 0; i<sensor_fusion.size(); i++)
            // {
            //   sensor_fusion[i][3] = MPH_to_mps(sensor_fusion[i][3]);
            //   sensor_fusion[i][4] = MPH_to_mps(sensor_fusion[i][4]);
            // }
            // cout<<"-----------------"<<endl;
            // cout<<"Current State - "<<current_state<<endl;
            // cout<<"car s,d "<<car_s<<'\t'<<car_d<<endl;

            // cout<<"car x, y, yaw "<<car_x<<'\t'<<car_y<<'\t'<<car_yaw<<endl;
            // if(previous_path_x.size() == 0)
            // {
            //   vector<double> xy = getXY(139.622, 6.16483, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   cout<<"xy "<<xy[0]<<'\t'<<xy[1]<<endl;
            //   // vector<double> sd = getFrenet(909.48, 1128.67, 0.0, map_waypoints_x, map_waypoints_y);
            //   // cout<<"sd "<<sd[0]<<'\t'<<sd[1]<<endl;
            //   vector<double> vel = getF_velocity(xy[0], xy[1], 0.1, 10.0*cos(0.1), 10.0*sin(0.1), map_waypoints_x, map_waypoints_y);
            //   cout<<"Frenet velocity "<<vel[0]<<'\t'<<vel[1]<<endl;  
            //   // return 0;
            // }
            // prev_s.clear();
            // prev_d.clear();

            // for(int i = 0; i<previous_path_x.size(); i++)
            // {
            //   vector<double> temp = get
            // }

            //Declaring vectors to keep the latest state, at time when ego vehicle has covered the previous trajectory, of the vehicles.
            //These vectors will be used to feed the state of the vehicles to the behavior planner and trajectory generator.
            cout<<"----------------------"<<endl;
            vector<double> latest_ego_state;
            vector<vector<double>> latest_sensor_fusion;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            vector<double> temp_s;
            vector<double> temp_d;

            int prev_size = previous_path_x.size();
            // cout<<"previous path size - "<<prev_size<<endl;
            int num_waypoints = (time_for_maneuver/0.02);
            // int num_waypoints_from_previous_path = prev_size;            
            // int num_waypoints_from_previous_path = min(previous_path_waypoints, prev_size);
            int num_waypoints_from_previous_path = min(prev_size,previous_path_waypoints);
            // cout<<"Num waypoints from previous path - "<<num_waypoints_from_previous_path<<endl;

            // cout<<"car s,d "<<car_s<<'\t'<<car_d<<endl;
            // 1. Modifying the data from the simulator to store in the vehicle class.
            
                // 1.a. When the vehicle starts moving from rest.
            if((previous_path_x.size() == 0 && car_speed == 0) || (previous_path_x.size() == 0))
            {
              num_waypoints_from_previous_path = 0;
              latest_ego_state = {car_s, 0, 0, car_d, 0, 0};

              //To filter out the vehicles which are far away from the ego vehicle and those giving back abrupt d values.
              for(int i = 0;i<sensor_fusion.size(); i++)
              {
                // cout<<"CHECK !"<<endl;
                double v_x = sensor_fusion[i][3];
                double v_y = sensor_fusion[i][4];
                if(sensor_fusion[i][6] > 13.0 || sensor_fusion[i][6] < 0.0 || fabs(double(sensor_fusion[i][5]) - car_s) >= SEARCH_DISTANCE) continue; 
                latest_sensor_fusion.push_back(sensor_fusion[i]);
              }
            }

            //1.b. Vehicle already in motion. 
            else
            {
              // return 0;
              // cout<<"-----------Debugging else condition of previous path size------------------"<<endl;            
              for(int i = 0; i<num_waypoints_from_previous_path; i++)
              {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
              // cout<<"next x vals size = "<<next_x_vals.size()<<endl;

              for(int i = prev_s.size() - previous_path_x.size(); i<prev_s.size(); i++)
              {
                if(temp_s.size() < num_waypoints_from_previous_path)
                {
                  temp_s.push_back(prev_s[i]);
                  temp_d.push_back(prev_d[i]);                  
                }
              }
              // cout<<"Temp S size = "<<temp_s.size()<<endl;
              // vector<double> xy = getXY(temp_s[temp_s.size()-1], temp_d[temp_s.size()-1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
              // cout<<"-----------CHECKING LAST WAYPOINTS----------------"<<endl;
              // cout<<"FROM NEXT X VALS = "<<next_x_vals[next_x_vals.size()-1]<<", "<<next_y_vals[next_y_vals.size()-1]<<endl;
              // cout<<"FROM TEMP (S,D) = "<<xy[0]<<", "<<xy[1]<<endl;
              prev_s = temp_s;
              prev_d = temp_d;

              // int last_index = (prev_s.size() - previous_path_x.size()) + (num_waypoints_from_previous_path - 1);
              // int last_index = previous_path_x.size() - 1;
              // cout<<"LAST INDEX = "<<last_index<<endl;
              // cout<<"FIRST CHECK"<<endl;
              // cout<<"prev_s size ="<<prev_s.size()<<endl;
              int last_index = prev_s.size() - 1;
              double latest_s = prev_s[last_index];
              double latest_d = prev_d[last_index];
              double pen_s = car_s;//penultimate s value
              double pen_d = car_d;
              double latest_3_s;
              double latest_3_d;
              

              vector<double> latest_xy = getXY(latest_s, latest_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              // cout<<"DEBUGGING---------"<<endl;
              // cout<<"LATEST X, Y = "<<next_x_vals[next_x_vals.size()-1]<<", "<<next_y_vals[next_y_vals.size()-1]<<endl;
              // cout<<"LATEST X, Y  from SIM = "<<latest_xy[0]<<", "<<latest_xy[1]<<endl;

              if(last_index > 0)
              {
                pen_s = prev_s[last_index-1];
                pen_d = prev_d[last_index-1];
              }

              double latest_velocity_s;
              double latest_velocity_d;
              double latest_velocity;

              if(abs(latest_s - pen_s) > 500.0)//comparing to a large number.
              {
                latest_velocity_s = (fmod(max(latest_s, max_s - pen_s), max_s) + min(latest_s, max_s - pen_s))/timestep;
              }
              else latest_velocity_s = (latest_s - pen_s)/timestep;

              latest_velocity_d = (latest_d - pen_d)/timestep;

              latest_velocity = sqrt(latest_velocity_s*latest_velocity_s + latest_velocity_d*latest_velocity_d);
              // cout<<"Printing the latest velocity = "<<latest_velocity<<endl;
              
              double pen_velocity_s = latest_velocity_s;
              double pen_velocity_d = latest_velocity_d;

              if(last_index == 1)
              {
                latest_3_s = car_s;
                latest_3_d = car_d;
              }
              else if(last_index > 1) //acc = -2.53, s_dot = 19.44, 
              {
                latest_3_s = prev_s[last_index-2];
                latest_3_d = prev_d[last_index-2];
              }

              if(abs(pen_s - latest_3_s) > 500.0) pen_velocity_s = (fmod(max(pen_s, max_s - latest_3_s), max_s) + min(pen_s, max_s - latest_3_s))/timestep;
              else pen_velocity_s = (pen_s - latest_3_s)/timestep;

              pen_velocity_d = (pen_d - latest_3_d)/timestep;

              double latest_acceleration_s = (latest_velocity_s - pen_velocity_s)/timestep;
              double latest_acceleration_d = (latest_velocity_d - pen_velocity_d)/timestep;
              // double latest_acceleration =  sqrt(latest_acceleration_s*latest_acceleration_s + latest_acceleration_d*latest_acceleration_d);
              
              // double delta_s = latest_s - pen_s;
              // if(abs(delta_s) > 500.0) delta_s = fmod(max(latest_s, max_s - pen_s), max_s) + min(latest_s, max_s - pen_s);

              // double latest_yaw = atan2(latest_d - pen_d, delta_s);

              latest_ego_state = {latest_s, latest_velocity_s, latest_acceleration_s, latest_d, latest_velocity_d, latest_acceleration_d};

              //Now we will predict the states of the traffic vehicles at the time when the ego vehicle has completed the last trajectory.
              // double delta_t = (num_waypoints_from_previous_path - (prev_s.size() - previous_path_x.size())) * 0.02;//time when trajectory is completed - present time.
              // double delta_t = num_waypoints_from_previous_path * 0.02;
              double delta_t = timestep*num_waypoints_from_previous_path;
              for(int i = 0; i<sensor_fusion.size(); i++)
              {
                // cout<<"CHECK !"<<endl;
                //This sensor fusion gives us the position of the traffic vehicles after the execution of few waypoints. So
                double v_x = sensor_fusion[i][3];
                double v_y = sensor_fusion[i][4];
                double v = sqrt(v_x*v_x + v_y*v_y);
                double new_traffic_s = sensor_fusion[i][5];
                // new_traffic_s+= v*delta_t;

                if(sensor_fusion[i][6] > 13.0 || sensor_fusion[i][6] < 0.0 || fabs(new_traffic_s - car_s) >= SEARCH_DISTANCE) continue; 
                vector<double> l  = sensor_fusion[i];
                // l[1]+= l[3]*delta_t;
                // l[2]+= l[4]*delta_t;

                // vector<double> frenet_coordinates = getFrenet(l[1], l[2], atan2(l[4], l[3]), map_waypoints_x, map_waypoints_y);
                l[5]+= v*delta_t;
                latest_sensor_fusion.push_back(l);
              }
              // prev_s = temp_s;
              // prev_d = temp_d;
            }
            // cout<<"_________LATEST EGO STATE______________________"<<endl;
            // cout<<"Latest ego state "<<latest_ego_state[0]<<'\t'<<latest_ego_state[1]<<'\t'<<latest_ego_state[2]<<'\t'<<latest_ego_state[3]<<endl;
            // cout<<endl;
            // //------------------------- 2. Initializing the map in the vehicle class which stores state information of all the vehicles.----------------------------------
            
            Vehicle vehicle(latest_ego_state);//s, s_dot, s_dot_dot, d, d_dot, d_dot_dot.
            // cout<<"Updated State of traffic "<<endl;
            cout<<"LATEST SENSOR FUSION SIZE = "<<latest_sensor_fusion.size()<<endl;
            for(int i = 0; i<latest_sensor_fusion.size(); i++)
            {
              vector<double> l = latest_sensor_fusion[i];//id, x, y, vx, vy, s, d.
              double velocity = sqrt(l[3]*l[3] + l[4]*l[4]);
              // vector<double> traffic_frenet_vel = getF_velocity(l[1], l[2], atan2(l[4], l[3]), l[3], l[4], map_waypoints_x, map_waypoints_y);
              vehicle.vehicles.insert(pair<int, vector<double>>(l[0],{l[5], velocity, 0, l[6], 0, 0}));
              // cout<<l[0]<<'\t'<<l[5]<<'\t'<<l[6]<<'\t'<<traffic_frenet_vel[0]<<'\t'<<traffic_frenet_vel[1]<<endl;
            }

            // //--------------------------3. Creating a finite state machine and  deciding the next vehicle state based on the predictions--------------------------------
            //-----------------START-----------
            //-----------EVALUATING CAR'S VELOCITY AND ACCELERATION IN FRENET SYSTEM----------------//
            // double latest_s = car_s;
            // double latest_d = car_d;
            // double latest_2_s = car_s;
            // double latest_2_d = car_d;
            // double latest_3_s = car_s;
            // double latest_3_d = car_d;
            // double latest_velocity_s = 0;
            // double latest_velocity_d = 0;
            // double latest_2_velocity_s = 0;
            // double latest_2_velocity_d = 0;
            // double latest_acceleration_s = 5.0;
            // double latest_acceleration_d = 0;

            // if(num_waypoints_from_previous_path != 0) 
            // {
            //   latest_3_s = prev_s[0];
            //   latest_2_s = prev_s[1];
            //   latest_s = prev_s[2];
            //   latest_3_d = prev_d[0];
            //   latest_2_d = prev_d[1];
            //   latest_d = prev_d[2];

            //   latest_2_velocity_s = (latest_2_s - latest_3_s)/timestep;
            //   latest_velocity_s = (latest_s - latest_2_s)/timestep;
            //   latest_2_velocity_d = (latest_2_d - latest_3_d)/timestep;
            //   latest_velocity_d = (latest_d - latest_2_d)/timestep;
            //   latest_acceleration_s = (latest_velocity_s - latest_2_velocity_s)/timestep;
            //   latest_acceleration_d = (latest_velocity_d - latest_2_velocity_d)/timestep;
            // }


            FSM fsm;
            vector<vector<double>> original_goal_state = fsm.next_state(vehicle, current_state, current_lane);
            
            // vector<vector<double>> temp_original_goal_state;
            // // temp_original_goal_state.push_back({latest_s, latest_velocity_s, latest_acceleration_s, latest_d, latest_velocity_d, latest_acceleration_d});
            // temp_original_goal_state.push_back(original_goal_state[0]);
            // temp_original_goal_state.push_back(original_goal_state[1]);
            // temp_original_goal_state[1][0]+=2.0;

            vector<vector<vector<double>>> goal_states = fsm.generate_multiple_goal_states(original_goal_state);
            
//-------------------------END----------------
            // // 4. Generating a JMT between the start and goal states from the next state().
            // double temp_t = 1.4;
            // if(num_waypoints_from_previous_path == 0) temp_t = dt;
            generate_trajectory traj(TRAJECTORY_GENERATION_TIME);

            vector<vector<vector<double>>> jmt_waypoints = fsm.best_state_waypoints(original_goal_state, vehicle, traj, goal_states, map_waypoints_s, map_waypoints_x, map_waypoints_y, num_waypoints, num_waypoints_from_previous_path, current_state);
            //Selecting jmt waypoints after the last waypoint from previous path. Doing it by counting the number of waypoints.

            // cout<<"Modified SOURCE STATE(S,D) = "<<original_goal_state[0][0]<<", "<<original_goal_state[0][3]<<endl;
            // cout<<"GOAL STATE(S,D) = "<<original_goal_state[1][0]<<", "<<original_goal_state[1][3]<<endl;

            // vector<vector<vector<double>>> jmt_waypoints_mod = jmt_waypoints;
            // if(num_waypoints_from_previous_path !=0)
            // {
            //   jmt_waypoints_mod = vector<vector<vector<double>>>(jmt_waypoints.begin()+previous_path_x.size()-2-1, jmt_waypoints.end());
            //   // cout<<""
            // }
            //-------------------------------CHANGE MADE----------------------------------------

            // vector<vector<vector<double>>> jmt_waypoints = best_state_waypoints(vehicle, traj, goal_states, map_waypoints_s, map_waypoints_x, map_waypoints_y, num_waypoints, num_waypoints_from_previous_path, current_state);

            // -------TRAJECTORY DEBUGGING------------

            // cout<<"TRAJECTORY:"<<endl;
            // cout<<"   start------------goal"<<endl;
            // cout<<"s  "<<trajectory[0][0]<<"       "<<trajectory[1][0]<<endl;
            // cout<<"d  "<<trajectory[0][3]<<"       "<<trajectory[1][3]<<endl;

            // cout<<"Coefficients:"<<endl;
            // for(int i = 0; i<coeffs.size();i++)
            // {
            //   cout<<"row - "<<i<<endl;
            //   for(int j = 0; j<coeffs[0].size(); j++)
            //   {
            //     cout<<coeffs[i][j]<<", ";
            //   }
            //   cout<<endl;
            // }
//---------------------------------------------------------------S
            
//---------------This piece works good-------------------//
            // vector<vector<double>> path = get_new_waypoints(trajectory, vehicle.vehicles.find(ego_id)->second);
            // for(int i = 0; i<path[0].size(); i++)
            // // for(int i = 0; i<num_waypoints - num_waypoints_from_previous_path; i++)
            // {
            //   // if(i>0 && path[0][i] <= check_value) continue;
            //   // check_value = path[0][i];
            //   // if(i < 100) cout<<"new s,d "<<path[0][i]<<'\t'<<path[1][i]<<endl;
            //   prev_s.push_back(path[0][i]);
            //   prev_d.push_back(path[1][i]);
            //   vector<double> xy = getXY(path[0][i], path[1][i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   // cout<<"xy "<<xy[0]<<'\t'<<xy[1]<<endl;
            //   next_x_vals.push_back(xy[0]);
            //   next_y_vals.push_back(xy[1]);
            // }
//------------------END-----------------------------------//
          

            // cout<<"-----------Next vals---------------"<<endl;
            // for(int i = 0; i<next_x_vals.size(); i++) cout<<"xy "<<next_x_vals[i]<<'\t'<<next_y_vals[i]<<endl;
            // cout<<"next x vals size "<<next_x_vals.size()<<endl;

            // 5. creating waypoints on the above trajectory for the vehicle to follow.            
            
            vector<double> jmt_x;
            vector<double> jmt_y;
            //   //Storing JMT waypoints.
            // cout<<"-------------------------Printing new S and D---------------------------"<<endl;
            // for(double t = 0.0; t<=time_for_maneuver; t+=timestep)
            // for(int i = 1; i<= num_waypoints - num_waypoints_from_previous_path; i++)
            // {
              // double t = i*timestep;
              // vector<vector<double>> jmt_waypoint = generate_JMT_waypoints(coeffs, t, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              // cout<<"s,d "<<jmt_waypoint[1][0]<<'\t'<<jmt_waypoint[1][1]<<endl;
            // cout<<"Printing JMT waypoints->"<<endl;
            // cout<<"BEFORE"<<endl;
            // cout<<"prev s size = "<<prev_s.size()<<endl;
            // cout<<"JMT WAYPOINTS SIZE  = "<<jmt_waypoints.size()<<endl;
            // if(num_waypoints_from_previous_path!=0)
            // {
            //   cout<<"-------LAST CHECK --------------------------"<<endl;
            //   cout<<"PREV (S,D) LAST WAYPOINTS = "<<prev_s[prev_s.size()-1]<<", "<<prev_d[prev_d.size()-1]<<endl;
            //   cout<<"JMT WAYPOINTS FIRST WAYPOINT = "<<jmt_waypoints[0][1][0]<<", "<<jmt_waypoints[0][1][1]<<endl;
            // }
            for(int i = 0; i < num_waypoints - num_waypoints_from_previous_path; i++)
            {
              vector<vector<double>> jmt_waypoint = jmt_waypoints[i];
              // cout<<jmt_waypoint[1][0]<<", "<<jmt_waypoint[1][1]<<endl;
              // cout<<"x,y = "<<jmt_waypoint[0][0]<<", "<<jmt_waypoint[0][1]<<endl;
              jmt_x.push_back(jmt_waypoint[0][0]);
              jmt_y.push_back(jmt_waypoint[0][1]);
              prev_s.push_back(jmt_waypoint[1][0]);
              prev_d.push_back(jmt_waypoint[1][1]);                
            }
            cout<<endl;
            // cout<<"prev s size = "<<prev_s.size()<<endl;
            //   //Pushing JMT waypoints to next_vals after some processing like fitting a polynomial with them because map waypoints are very sparse so getXY does not yield good results.
            // // cout<<"-------------------------New Path starts--------------------------"<<endl;
            // int check_flag = 1;
            // double check_value = jmt_x[0];
            for(int i = 0; i<jmt_x.size(); i++)
            {
              // if(i > 0 && jmt_x[i] <= check_value)
              // {
              //   continue;
              //   // cout<<"index when decrement starts - "<<num_waypoints_from_previous_path + i<<endl;
              //   // check_flag = 0;
              // }
              // check_value = jmt_x[i];
              next_x_vals.push_back(jmt_x[i]);
              next_y_vals.push_back(jmt_y[i]);
              // cout<<"x, y "<<'\t'<<jmt_x[i]<<'\t'<<jmt_y[i]<<endl;
            }
            // cout<<"--------------"<<endl;
            // cout<<"prev s size = "<<prev_s.size()<<endl;
            // cout<<"Printing New Path vals - (S,D) "<<endl;
            // for(int i = 0; i<prev_s.size();i++)
            // {
            //   cout<<prev_s[i]<<" "<<prev_d[i]<<endl;
            // }
            // bool lcl_flag;
            // if(current_state.compare("LCL") == 0 || current_state.compare("LCR") == 0)
            // {
            //   lcl_flag = true;
            //   cout<<"------------"<<endl;
            //   cout<<"Printing New Path vals - (S,D) "<<endl;

            //   for(int i = 0; i<prev_s.size();i++)
            //   {
            //     cout<<prev_s[i]<<" "<<prev_d[i]<<endl;
            //   }
            // }
            // if(lcl_flag)
            // {
            //   cout<<"------------"<<endl;
            //   cout<<"Printing New Path vals - (S,D) "<<endl;

            //   for(int i = 0; i<prev_s.size();i++)
            //   {
            //     cout<<prev_s[i]<<" "<<prev_d[i]<<endl;
            //   }
            // }
            // if(previous_path_x.size() == 0)
            // {
            //   cout<<"---------Printing First Path----------"<<endl;
            //   for(int i = 0; i<next_x_vals.size(); i++)
            //   {
            //     cout<<"x,y "<<next_x_vals[i]<<'\t'<<next_y_vals[i]<<endl;
            //   }
            // }

            json msgJson;            
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
