#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <map>
#include <string>

static double max_s = 6945.554;
static int ego_id = -1;
static int d_min = 0;
static int d_max = 2;
static int NUM_LANES = 3;
static std::map<std::string, int> stoi_state = {{"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"PLCR", 1}, {"LCR", 1}};
static double dt = 1.0;//time duration for predicting a goal state. It decides the length of the path.
static double time_for_maneuver = dt;// time duration to cover the trajectory.
static double timestep = 0.02;
static double SEARCH_DISTANCE= 60.0;//(in meters) Search distance for the vehicles in a lane. It decides whether to take a vehicle in consideration or not.
static double BUFFER_DISTANCE = 20.0;//(in meters) Distance to maintain between the vehicles.
static double MAX_SPEED = (49.5 * 1.60934 * 5.0/18.0);//in m/s.
static double MAX_ACCELERATION = 10;//in m/s^2.
static double MAX_JERK = 10.0;//in m/s^3
static double MAX_COST = 1e10;
static int previous_path_waypoints = 0.9*(dt/timestep);//42
static double TRAJECTORY_GENERATION_TIME = 2.0;
// static double BUFFER_TIME = 0.75;//0.75//This will be used along with current vehicle velocity to calculate the buffer distance.
// static double SEARCH_TIME = 2.0;//This will be used along wtih current vehicle velocity to calculate the search distance.
#endif

//Changes needed.
//1. Do not change lane if fastest lane is not too fast.
//2. Velocity and acceleration violence at each trajectory point.