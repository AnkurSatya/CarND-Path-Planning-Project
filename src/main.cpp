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
#include "helper.h"
#include "behavior_planner.h"
#include "trajectory_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  double max_s = 6945.554;

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
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];//in degrees.
          	double car_speed = j[1]["speed"];
            //Insert car's state as id, s, s_dot, d, d_dot.

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];//returns a vector of strings
            cout<<"previous path size - "<<previous_path_x.size()<<endl;
            
            //Declaring vectors to keep the latest state, at time when ego vehicle has covered the previous trajectory, of the vehicles.
            //These vectors will be used to feed the state of the vehicles to the behavior planner and trajectory generator.

            vector<double> latest_ego_state;
            vector<vector<double>> latest_sensor_fusion;

            // 1. Modifying the data from the simulator to store in the vehicle class.
            
            // 1.a. When the vehicle starts moving from rest.
            if((previous_path_x.size() == 0 && car_speed == 0) || (previous_path_x.size() == 0))
            {
              vector<double> frenet_vel = getF_velocity(car_x, car_y, car_yaw, car_speed * cos(car_yaw), car_speed * sin(car_yaw), map_waypoints_x, map_waypoints_y);
              latest_ego_state = {car_s, frenet_vel[0], 0, car_d, frenet_vel[1], 0};
              //To filter out the vehicles which are far away from the ego vehicle and those giving back abrupt d values.
              for(int i = 0;i<sensor_fusion.size(); i++)
              {
                // cout<<"Vehicle - "<<sensor_fusion[i][0]<<", s-> "<<sensor_fusion[i][5]<<", d-> "<<sensor_fusion[i][6]<<endl;
                if(sensor_fusion[i][6] > 13.0 || sensor_fusion[i][6] < 0.0 || fabs(double(sensor_fusion[i][5]) - latest_ego_state[0]) > 1000) continue; 
                latest_sensor_fusion.push_back(sensor_fusion[i]);
              }
            }

            //1.b. Vehicle already in motion. 
            else
            {
              // std::this_thread::sleep_for(std::chrono::milliseconds(6000));

              // All the below values are under the assuming the current position is the goal position from the previous state.
              int last_index = previous_path_x.size() - 1;
              double latest_x = previous_path_x[last_index];
              double latest_y = previous_path_y[last_index];
              double pen_x = car_x;//penultimate x value in the previous_path_x.
              double pen_y = car_y;
              if(previous_path_x.size() > 1)
              {
                pen_x = previous_path_x[last_index - 1];
                pen_y = previous_path_y[last_index - 1];
              }
              double latest_velocity = distance(latest_x, latest_y, pen_x, pen_y)/0.02;

              double pen_velocity = latest_velocity;
              if(previous_path_x.size() == 2)
              {
                pen_velocity = distance(pen_x, pen_y, car_x, car_y)/0.02;
              }
              else if(previous_path_x.size() >2)
              {
                pen_velocity = distance(pen_x, pen_y, previous_path_x[last_index - 2], previous_path_y[last_index - 2])/0.02;
              }
              double latest_acceleration = (latest_velocity - pen_velocity)/0.02;
              double latest_yaw = atan2(latest_y - pen_y, latest_x - pen_x);

              vector<double> frenet_vel = getF_velocity(latest_x, latest_y, latest_yaw, latest_velocity*cos(latest_yaw), latest_velocity * sin(latest_yaw), map_waypoints_x, map_waypoints_y);
              latest_ego_state = {end_path_s, frenet_vel[0], latest_acceleration, end_path_d, frenet_vel[1], 0.0};

              //Now we will predict the states of the traffic vehicles at the time when the ego vehicle has completed the last trajectory.

              double delta_t = previous_path_x.size() * 0.02;//time when trajectory is completed - present time.
              for(int i = 0; i<sensor_fusion.size(); i++)
              {
                if(sensor_fusion[i][6] > 13.0 || sensor_fusion[i][6] < 0.0 || fabs(double(sensor_fusion[i][5]) - latest_ego_state[0]) > 1000) continue; 
                vector<double> l  = sensor_fusion[0];
                l[1]+= l[3]*delta_t;
                l[2]+= l[4]*delta_t;

                vector<double> frenet_coordinates = getFrenet(l[1], l[2], atan2(l[4], l[3]), map_waypoints_x, map_waypoints_y);
                l[5] = frenet_coordinates[0];
                l[6] = frenet_coordinates[1];
                latest_sensor_fusion.push_back(l);
              }
            }

            // 2. Initializing the map in the vehicle class which stores state information of all the vehicles.
            
            Vehicle vehicle(latest_ego_state);//s, s_dot, s_dot_dot, d, d_dot, d_dot_dot.
            
            for(int i = 0; i<latest_sensor_fusion.size(); i++)
            {
              vector<double> l = latest_sensor_fusion[i];//id, x, y, vx, vy, s, d.
              vector<double> traffic_frenet_vel = getF_velocity(l[1], l[2], atan2(l[4], l[3]), l[3], l[4], map_waypoints_x, map_waypoints_y);
              vehicle.vehicles.insert(pair<int, vector<double>>(l[0],{l[5], traffic_frenet_vel[0], 0, l[6], traffic_frenet_vel[1], 0}));
            }

            // 3. Creating a finite state machine and  deciding the next vehicle state based on the predictions.
            
            FSM fsm;
            vector<vector<double>> trajectory = fsm.next_state(vehicle, "KL");

            // 4. Generating a JMT between the start and goal states from the next state().
            double time_for_maneuver = 2.0;// 1.5 seconds for following the trajectory. 
            generate_trajectory traj(time_for_maneuver);
            vector<vector<double>> coeffs = traj.generate_JMT(trajectory);

            // for(int i = 0; i<coeffs.size(); i++)
            // {
            //   cout<<" row - "<<endl;
            //   for(int j = 0; j<coeffs[0].size(); j++)
            //   {
            //     cout<<"Coeffs - "<<coeffs[i][j]<<endl;
            //   }
            // }

            // 5. creating waypoints on the above trajectory for the vehicle to follow.

            double shortest_distance = distance(trajectory[0][0], trajectory[0][3], trajectory[1][0], trajectory[1][3]);
            int num_waypoints = 40;
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for(int i = 1; i<=num_waypoints; i++)
            {
              // double t = double(i)/double(num_waypoints);
              double t = i * 0.05;
              vector<double> cartesian = generate_xy_for_trajectory(coeffs, t, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(cartesian[0]);
              next_y_vals.push_back(cartesian[1]);
              // cout<<"X - "<<cartesian[0]<<endl;
              // cout<<"Y - "<<cartesian[1]<<endl;
              
            }

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
