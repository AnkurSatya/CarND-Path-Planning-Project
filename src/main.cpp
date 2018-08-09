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
  string current_state = "KL";

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &current_state, &prev_s, &prev_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            car_speed = (car_speed * 1.60934 * 5.0/18.0);//converting MPH to m/s.
            car_d = 10;
            //Insert car's state as id, s, s_dot, d, d_dot.

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];//returns a vector of strings

            cout<<"-----------------"<<endl;
            cout<<"car s,d "<<car_s<<'\t'<<car_d<<endl;

            cout<<"car x, y, yaw "<<car_x<<'\t'<<car_y<<'\t'<<car_yaw<<endl;
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

            //Declaring vectors to keep the latest state, at time when ego vehicle has covered the previous trajectory, of the vehicles.
            //These vectors will be used to feed the state of the vehicles to the behavior planner and trajectory generator.

            vector<double> latest_ego_state;
            vector<vector<double>> latest_sensor_fusion;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            int prev_size = previous_path_x.size();
            int num_waypoints = 50;
            int num_waypoints_from_previous_path = prev_size;            
            // int num_waypoints_from_previous_path = min(40, prev_size);
            cout<<"Num waypoints from previous path - "<<num_waypoints_from_previous_path<<endl;

            cout<<"car s,d "<<car_s<<'\t'<<car_d<<endl;
            // 1. Modifying the data from the simulator to store in the vehicle class.
            
                // 1.a. When the vehicle starts moving from rest.
            if((previous_path_x.size() == 0 && car_speed == 0) || (previous_path_x.size() == 0))
            {
              // vector<double> frenet_vel = getF_velocity(car_x, car_y, car_speed * cos(car_yaw), car_speed * sin(car_yaw), map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
              vector<double> frenet_vel = getF_velocity(car_x, car_y, car_yaw, car_speed * cos(car_yaw), car_speed*sin(car_yaw), map_waypoints_x, map_waypoints_y);
              latest_ego_state = {car_s, frenet_vel[0], 0, car_d, frenet_vel[1], 0};

              //To filter out the vehicles which are far away from the ego vehicle and those giving back abrupt d values.
              for(int i = 0;i<sensor_fusion.size(); i++)
              {
                if(sensor_fusion[i][6] > 13.0 || sensor_fusion[i][6] < 0.0 || fabs(double(sensor_fusion[i][5]) - latest_ego_state[0]) > SEARCH_DISTANCE) continue; 
                latest_sensor_fusion.push_back(sensor_fusion[i]);
              }
            }

            //1.b. Vehicle already in motion. 
            else
            {
              cout<<"-----------Debugging else condition of previous path size------------------"<<endl;            
              // cout<<"------------------Previous path starts----------------------"<<endl;
              for(int i = 0; i<num_waypoints_from_previous_path; i++)
              {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
                // cout<<"x, y "<<'\t'<<previous_path_x[i]<<'\t'<<previous_path_y[i]<<endl;
              }

              // int last_index = previous_path_x.size() - 1;
              int last_index = num_waypoints_from_previous_path-1;//ADDED
              double latest_x = previous_path_x[last_index];
              double latest_y = previous_path_y[last_index];
              cout<<"latest x, y "<<latest_x<<'\t'<<latest_y<<endl;
              double pen_x = car_x;//penultimate x value in the previous_path_x.
              double pen_y = car_y;
              
              if(num_waypoints_from_previous_path > 1)
              {
                pen_x = previous_path_x[last_index - 1];
                pen_y = previous_path_y[last_index - 1];
              }
              cout<<"pen_x, pen_y "<<pen_x<<'\t'<<pen_y<<endl;
              double latest_velocity = distance(latest_x, latest_y, pen_x, pen_y)/0.02;

              double pen_velocity = latest_velocity;
              if(num_waypoints_from_previous_path == 2)//ADDED
              {
                pen_velocity = distance(pen_x, pen_y, car_x, car_y)/0.02;
              }
              else if(num_waypoints_from_previous_path > 2)
              {
                pen_velocity = distance(pen_x, pen_y, previous_path_x[last_index - 2], previous_path_y[last_index - 2])/0.02;
                cout<<"second last x,y "<<previous_path_x[last_index-2]<<'\t'<<previous_path_y[last_index-2]<<endl;
              }
              cout<<"latest_velocity, pen_velocity "<<latest_velocity<<'\t'<<pen_velocity<<endl;
              double latest_acceleration = (latest_velocity - pen_velocity)/0.02;
              double latest_yaw = atan2(latest_y - pen_y, latest_x - pen_x);
              cout<<"latest_acceleration, latest_yaw "<<latest_acceleration<<'\t'<<latest_yaw<<endl;

              // vector<double> frenet_vel = getF_velocity(latest_x, latest_y, latest_velocity*cos(latest_yaw), latest_velocity * sin(latest_yaw), map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
              vector<double> frenet_vel = getF_velocity(latest_x, latest_y, latest_yaw, latest_velocity*cos(latest_yaw), latest_velocity*sin(latest_yaw), map_waypoints_x, map_waypoints_y);
              vector<double> end_path_frenet = getFrenet(latest_x, latest_y, latest_yaw, map_waypoints_x, map_waypoints_y);
              latest_ego_state = {end_path_frenet[0], frenet_vel[0], latest_acceleration, end_path_frenet[1], frenet_vel[1], 0.0};
              cout<<"frenet_velocity s,d "<<frenet_vel[0]<<'\t'<<frenet_vel[1]<<endl;
              cout<<"end path frenet s,d "<<end_path_frenet[0]<<'\t'<<end_path_frenet[1]<<endl;

              //Now we will predict the states of the traffic vehicles at the time when the ego vehicle has completed the last trajectory.
              double delta_t = num_waypoints_from_previous_path * 0.02;//time when trajectory is completed - present time.
              for(int i = 0; i<sensor_fusion.size(); i++)
              {
                if(sensor_fusion[i][6] > 13.0 || sensor_fusion[i][6] < 0.0 || fabs(double(sensor_fusion[i][5]) - latest_ego_state[0]) >= SEARCH_DISTANCE) continue; 
                vector<double> l  = sensor_fusion[0];
                l[1]+= l[3]*delta_t;
                l[2]+= l[4]*delta_t;

                vector<double> frenet_coordinates = getFrenet(l[1], l[2], atan2(l[4], l[3]), map_waypoints_x, map_waypoints_y);
                l[5] = frenet_coordinates[0];
                l[6] = frenet_coordinates[1];
                latest_sensor_fusion.push_back(l);
              }
            }
            cout<<"Latest ego state "<<latest_ego_state[0]<<'\t'<<latest_ego_state[1]<<'\t'<<latest_ego_state[2]<<'\t'<<latest_ego_state[3]<<endl;

            //------------------------- 2. Initializing the map in the vehicle class which stores state information of all the vehicles.----------------------------------
            

            Vehicle vehicle(latest_ego_state);//s, s_dot, s_dot_dot, d, d_dot, d_dot_dot.
  
            for(int i = 0; i<latest_sensor_fusion.size(); i++)
            {
              vector<double> l = latest_sensor_fusion[i];//id, x, y, vx, vy, s, d.
              vector<double> traffic_frenet_vel = getF_velocity(l[1], l[2], atan2(l[4], l[3]), l[3], l[4], map_waypoints_x, map_waypoints_y);
              vehicle.vehicles.insert(pair<int, vector<double>>(l[0],{l[5], traffic_frenet_vel[0], 0, l[6], traffic_frenet_vel[1], 0}));
            }

            // //--------------------------3. Creating a finite state machine and  deciding the next vehicle state based on the predictions--------------------------------
            //-----------------START-----------
            FSM fsm;
            vector<vector<double>> trajectory = fsm.next_state(vehicle, current_state);
//-------------------------END----------------
            // // 4. Generating a JMT between the start and goal states from the next state().
            // generate_trajectory traj(time_for_maneuver);

            //-------------------------------CHANGE MADE----------------------------------------
            // vector<vector<double>> coeffs = traj.generate_JMT(trajectory);

            //-------TRAJECTORY DEBUGGING------------

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
            vector<vector<double>> path = get_new_waypoints(trajectory, vehicle.vehicles.find(ego_id)->second);
            for(int i = 0; i<path[0].size(); i++)
            // for(int i = 0; i<num_waypoints - num_waypoints_from_previous_path; i++)
            {
              prev_s.push_back(path[0][i]);
              prev_d.push_back(path[1][i]);
              vector<double> xy = getXY(path[0][i], path[1][i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
              // cout<<"xy "<<xy[0]<<'\t'<<xy[1]<<endl;
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
            }


            // cout<<"-----------Next vals---------------"<<endl;
            // for(int i = 0; i<next_x_vals.size(); i++) cout<<"xy "<<next_x_vals[i]<<'\t'<<next_y_vals[i]<<endl;
            // cout<<"next x vals size "<<next_x_vals.size()<<endl;

            // 5. creating waypoints on the above trajectory for the vehicle to follow.            
            // double timestep = 0.02;
            // int num_waypoints = (time_for_maneuver/timestep);
            // vector<double> jmt_x;
            // vector<double> jmt_y;
            //   //Storing JMT waypoints.
            // // cout<<"-------------------------Printing new S and D---------------------------"<<endl;
            // for(double t = 0.0; t<=time_for_maneuver; t+=timestep)
            // // for(int i = 1; i<= num_waypoints - num_waypoints_from_previous_path; i++)
            // {
            //   // double t = i*timestep;
            //   vector<double> jmt_waypoint = generate_xy_for_trajectory(coeffs, t, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   // cout<<"x,y "<<jmt_waypoint[0]<<'\t'<<jmt_waypoint[1]<<endl;
            //   jmt_x.push_back(jmt_waypoint[0]);
            //   jmt_y.push_back(jmt_waypoint[1]);

            //   // vector<double> vel_and_acc = trajectory_vel_and_acc(coeffs, t);//Just for testing. In the beginning of the path, major acceleration is in the x direction. The velocity and acceleration are calcultaed in s and d space and assigned to acc_x and vel_x for rough approximation.
            // }

            //   //Pushing JMT waypoints to next_vals after some processing like fitting a polynomial with them because map waypoints are very sparse so getXY does not yield good results.
            // // cout<<"-------------------------New Path starts--------------------------"<<endl;
            // int check_flag = 1;
            // double check_value = jmt_x[0];
            // for(int i = 0; i<jmt_x.size(); i++)
            // {
            //   // if(i > 0 && jmt_x[i] <= check_value)
            //   // {
            //   //   continue;
            //   //   // cout<<"index when decrement starts - "<<num_waypoints_from_previous_path + i<<endl;
            //   //   // check_flag = 0;
            //   // }
            //   // check_value = jmt_x[i];
            //   next_x_vals.push_back(jmt_x[i]);
            //   next_y_vals.push_back(jmt_y[i]);
            //   cout<<"x, y "<<'\t'<<jmt_x[i]<<'\t'<<jmt_y[i]<<endl;
            // }
            
            // cout<<"car x,y "<<car_x<<'\t'<<car_y<<endl;
            // cout<<"next initial x,y "<<next_x_vals[0]<<'\t'<<next_y_vals[0]<<endl;
            // cout<<"next final x,y "<<next_x_vals[next_x_vals.size()-1]<<'\t'<<next_y_vals[next_y_vals.size()-1]<<endl;
            // if(previous_path_y.size())
            // {
            //   cout<<"previous path initial x,y "<<previous_path_x[0]<<", "<<previous_path_y[0]<<endl;
            //   cout<<"previous path last x,y "<<previous_path_x[previous_path_x.size() - 1]<<", "<<previous_path_y[previous_path_y.size() - 1]<<endl;

            //   // vector<double> frenet_t_1 = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
            //   // cout<<"car s,d "<<frenet_t_1[0]<<", "<<frenet_t_1[1]<<endl;
            //   cout<<"car x,y "<<car_x<<'\t'<<car_y<<endl;
            // }
            // double temp_yaw = atan2(next_y_vals[1] - next_y_vals[0], next_x_vals[1] - next_x_vals[0]);
            // vector<double> frenet_t_2 = getFrenet(next_x_vals[0], next_y_vals[0], temp_yaw, map_waypoints_x, map_waypoints_y);

            // cout<<"Next vals initial s,d "<<frenet_t_2[0]<<", "<<frenet_t_2[1]<<endl;
            // cout<<"next vals initial x,y "<<next_x_vals[0]<<'\t'<<next_y_vals[0]<<endl;

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
//TO DO:
// Not able to use section of previous path and not able to compliment it with generating fewer points using the JMT trajectory.
// 1. Implement spline either using the JMT generated waypoints or generate using the source and destination and few other points.
// 2. Then Use section of previous path.