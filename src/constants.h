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
static double time_for_maneuver = 1.0;// time duration to cover the trajectory.
static double SEARCH_DISTANCE= 100.0;//(in meters) Search distance for the vehicles in a lane. It decides whether to take a vehicle in consideration or not.
static double BUFFER_DISTANCE = 5.0;//(in meters) Distance to maintain between the vehicles along the heading.
static double MAX_SPEED = (50.0 * 1.60934 * 5.0/18.0);//in m/s.
static double MAX_ACCELERATION = 10;//in m/s^2.
static double MAX_COST = 1e10;

#endif