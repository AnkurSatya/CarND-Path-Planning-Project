#include <vector>
#include "constants.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline/src/spline.h"
#include "trajectory_planner.h"
#include "behavior_planner.h"
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// double MPH_to_mps(double v)
// {
// 	return v * 1.60934 * 5.0/18.0;
// }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  	angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

int NextWaypoint_alt(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = min(2*pi() - angle, angle);

	if(angle > pi()/2)
	{
		closestWaypoint++;
		if(closestWaypoint == maps_x.size()){closestWaypoint =  0;}
	}
	return closestWaypoint;
 }

tk::spline generate_spline(vector<double> x, vector<double> y)
{
	tk::spline x_to_y;
	x_to_y.set_points(x, y);
	return x_to_y;
}

bool sort_by_s(vector<double> i, vector<double> j){return (i[2] < j[2]);}

void sort_map_waypoints(vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s)
{
	vector<vector<double>> map_wps;
	for(int i = 0; i<maps_x.size(); i++)
	{
		map_wps.push_back({maps_x[i], maps_y[i], maps_s[i]});
	}
	sort(map_wps.begin(), map_wps.end(), sort_by_s);

	maps_x.clear();
	maps_y.clear();
	maps_s.clear();

	for(int i = 0; i<map_wps.size(); i++)
	{
		maps_x.push_back(map_wps[i][0]);
		maps_y.push_back(map_wps[i][1]);
		maps_s.push_back(map_wps[i][2]);
	}
}
void sort_map_waypoints(vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s, vector<double>&maps_dx, vector<double> &maps_dy)
{
	vector<vector<double>> map_wps;

	for(int i = 0; i<maps_x.size(); i++)
	{
		map_wps.push_back({maps_x[i], maps_y[i], maps_s[i], maps_dx[i], maps_dy[i]});
	}
	sort(map_wps.begin(), map_wps.end(), sort_by_s);

	maps_x.clear();
	maps_y.clear();
	maps_s.clear();
	maps_dx.clear();
	maps_dy.clear();

	for(int i = 0; i<map_wps.size(); i++)
	{
		maps_x.push_back(map_wps[i][0]);
		maps_y.push_back(map_wps[i][1]);
		maps_s.push_back(map_wps[i][2]);
		maps_dx.push_back(map_wps[i][3]);
		maps_dy.push_back(map_wps[i][4]);
	}
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
// vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
// {
// 	//This is a very poorly designed mathematical formulation. The approximation is not fixed and increases with the curvature, map waypoint and vehicle position. Problem can be solved by using splines to generate more waypoints.
// 	int prev_wp = -1;

// 	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
// 	{
// 		prev_wp++;
// 	}

// 	int wp2 = (prev_wp+1)%maps_x.size();

// 	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
// 	// the x,y,s along the segment
// 	double seg_s = (s-maps_s[prev_wp]);

// 	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
// 	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

// 	double perp_heading = heading-pi()/2;

// 	double x = seg_x + d*cos(perp_heading);
// 	double y = seg_y + d*sin(perp_heading);

// 	return {x,y};
// }

// vector<double> getXY(double s, double d, vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_dx, vector<double> &maps_dy)
// {
// 	sort_map_waypoints(maps_x, maps_y, maps_s, maps_dx, maps_dy);
// 	tk::spline s_to_x = generate_spline(maps_s, maps_x);
// 	tk::spline s_to_y = generate_spline(maps_s, maps_y);
// 	tk::spline s_to_dx = generate_spline(maps_s, maps_dx);
// 	tk::spline s_to_dy = generate_spline(maps_s, maps_dy);	
// 	double x = s_to_x(s) + d*s_to_dx(s);
// 	double y = s_to_y(s) + d*s_to_dy(s);
// 	return {x,y};
// }

// Transform the speed from Cartesian v_x, v_y to Frenet Coordinates v_s, v_d.
vector<double> getF_velocity(double x, double y, double theta, double v_x, double v_y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int n_wp = NextWaypoint_alt(x, y, theta, maps_x, maps_y);
	int p_wp = n_wp - 1;
	if(n_wp == 0) p_wp = maps_x.size() - 1;

	double n_wp_x = maps_x[n_wp];
	double n_wp_y = maps_y[n_wp];
	double p_wp_x = maps_x[p_wp];
	double p_wp_y = maps_y[p_wp];

	double waypoints_heading = atan2((n_wp_y - p_wp_y), (n_wp_x - p_wp_x));
	double angle = fabs(waypoints_heading - theta);//angle between the vehicle heading and the vector connecting the previous and the next waypoints.
	angle = min(2*pi() - angle, angle);
	double velocity = pow(v_x*v_x + v_y*v_y, 0.5);
	double s_dot = velocity * cos(angle);
	double d_dot = velocity * sin(angle);
	return {s_dot, d_dot};
}

// vector<double> getF_velocity(double x, double y, double v_x, double v_y, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy)
// {
// 	int closest_waypoint = ClosestWaypoint(x, y, maps_x, maps_y);
// 	cout<<"closest_waypoint "<<closest_waypoint<<endl;
// 	double d_x = maps_dx[closest_waypoint];//these are x and y components of a unit vector representing 'd' of frenet space.
// 	double d_y = maps_dy[closest_waypoint];
// 	cout<<"dx,dy "<<d_x<<'\t'<<d_y<<endl;
// 	vector<double> s_unit_vector = {-d_y, d_x};
// 	vector<double> d_unit_vector = {d_x, d_y};
// 	double s_dot = v_x * s_unit_vector[0] + v_y * s_unit_vector[1];
// 	double d_dot = v_x * d_unit_vector[0] + v_y * d_unit_vector[1];
// 	// double s_dot = pow(v_x * v_x +v_y * v_y  - d_dot * d_dot, 0.5);
// 	// cout<<"Printing car vx and vy "<<v_x<<'\t'<<v_y<<endl;
// 	return {s_dot, d_dot};

// }

// vector<double> trajectory_vel_and_acc(vector<vector<double>> coeffs, double t)
// {
// 	vector<double> c_s = coeffs[0];
// 	vector<double> c_d = coeffs[1];

// 	double s_velocity = c_s[1] + 2*c_s[2]*t + 3*c_s[3]*t*t + 4*c_s[4]*t*t*t + 5*c_s[5]*t*t*t*t;
// 	double s_acceleration = 2*c_s[2] + 6*c_s[3]*t + 12*c_s[4]*t*t + 20*c_s[5]*t*t*t;

// 	double d_velocity = c_d[1] + 2*c_d[2]*t + 3*c_d[3]*t*t + 4*c_d[4]*t*t*t + 5*c_d[5]*t*t*t*t;
// 	double d_acceleration = 2*c_d[2] + 6*c_d[3]*t + 12*c_d[4]*t*t + 20*c_d[5]*t*t*t;

// 	return{pow(s_velocity*s_velocity + d_velocity*d_velocity, 0.5), pow(s_acceleration*s_acceleration + d_acceleration*d_acceleration, 0.5)};
// }


vector<vector<double>> upsample_map_waypoints(vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
	sort_map_waypoints(maps_x, maps_y, maps_s);
	tk::spline s_to_x = generate_spline(maps_s, maps_x);
	tk::spline s_to_y = generate_spline(maps_s, maps_y);

	vector<double> x_upsampled, y_upsampled, s_upsampled;
	for(double i = 0; i<=max_s; i++)// resampling every 1 meter.
	{	
		x_upsampled.push_back(s_to_x(i));
		y_upsampled.push_back(s_to_y(i));
		s_upsampled.push_back(double(i));
	}
	return {x_upsampled, y_upsampled, s_upsampled};
}


// vector<vector<double>> get_new_waypoints(vector<vector<double>> trajectory, vector<double> ego_state)
// {
// 	//Testing for keep lane trajectory. That is why d consist of same values. And d velocity is assumed to be zero.
// 	vector<double> start = trajectory[0];
// 	vector<double> goal = trajectory[1];
// 	// cout<<"goal d "<<goal[3]<<endl;

// 	vector<double> s;
// 	vector<double> d;

// 	double delta_s = (goal[0] - start[0])/10;
// 	for(double s_val = start[0]; s_val<=goal[0]; s_val+=delta_s)
// 	{
// 		s.push_back(s_val);
// 		d.push_back(start[3]);
// 	}

// 	tk::spline path = generate_spline(s,d);

// 	// path.set_points(s,d);

// 	int num_waypoints = 50;
// 	double s_increment = (goal[0] - start[0])/num_waypoints;

// 	vector<double> s_waypoints;
// 	vector<double> d_waypoints;

// 	double prev_s = start[0];
// 	double prev_velocity_s = ego_state[1];
// 	double prev_acceleration = ego_state[2];
// 	double t = 0.02;
// 	if(prev_velocity_s == 0)
// 	{
// 		prev_acceleration = MAX_ACCELERATION;
// 		prev_velocity_s = min(prev_acceleration * t, MAX_SPEED);
// 		// prev_velocity_s = MAX_SPEED;
// 	}
// 	// cout<<"------------------get new waypoints function-------------"<<endl;
// 	// cout<<"prev velocity"<<prev_velocity_s<<endl;
// 	double new_velocity_s = prev_velocity_s;
// 	double new_acceleration_s = prev_acceleration;
// 	prev_s+=prev_velocity_s*t + 0.5*t*t*prev_acceleration;//So that the last iteration's goal value, which is start[0] here, is not added again.

// 	// cout<<"----------New S and D---------"<<endl;
// 	int count = 0;
// 	while(prev_s < goal[0])
// 	{
// 		s_waypoints.push_back(prev_s);
// 		d_waypoints.push_back(path(prev_s));

// 		new_velocity_s+= prev_acceleration*t;
// 		new_velocity_s = min(MAX_SPEED, new_velocity_s);
// 		new_velocity_s = max(new_velocity_s, 0.2);
// 		// cout<<"new velocity "<<new_velocity_s<<endl;
// 		// new_velocity_s = MAX_SPEED;
// 		new_acceleration_s = min(MAX_ACCELERATION, (new_velocity_s - prev_velocity_s)/t);
// 		// cout<<"new acceleration "<<new_acceleration_s<<endl;
// 		prev_s+= new_velocity_s*t + 0.5 * t * t * new_acceleration_s;
// 		// prev_s+= new_velocity_s*t;
// 		prev_velocity_s = new_velocity_s;
// 		prev_acceleration = new_acceleration_s;
// 	}

// 	// cout<<"no. of new waypoints - "<<s_waypoints.size()<<endl;
// 	return {s_waypoints, d_waypoints};
// }

vector<vector<double>> test_behavior_planner(vector<double>ego_state, vector<vector<double>> sensor_fusion)
{
	vector<vector<double>> trajectory;
	trajectory.push_back(ego_state);
	double min_distance = max_s;
	double vehicle_ahead_id = -1;

	vector<double> new_state(ego_state.size(), 0.0);
	new_state[0] = ego_state[0];
	for(int i = 0; i<sensor_fusion.size(); i++)
	{
		if(int(sensor_fusion[i][6]/4) == int(ego_state[3]/4))
		{
			if(sensor_fusion[i][5] - ego_state[0] <= SEARCH_DISTANCE && sensor_fusion[i][5] > ego_state[0] && min_distance > (sensor_fusion[i][5] - ego_state[0]))
			{
				min_distance = sensor_fusion[i][5] - ego_state[0];
				vehicle_ahead_id = i; 
			}
		}
	}
	if(vehicle_ahead_id != -1)
	{
		new_state[1] = (sensor_fusion[vehicle_ahead_id][5] - BUFFER_DISTANCE - ego_state[0] - 0.5*ego_state[2]*dt*dt)/dt + pow(pow(sensor_fusion[vehicle_ahead_id][3],2)+pow(sensor_fusion[vehicle_ahead_id][4],2),0.5); 
		new_state[1] = min(MAX_SPEED, new_state[1]);
		new_state[2] = min(MAX_ACCELERATION, (new_state[1] - ego_state[1])/dt);
		new_state[0]+= new_state[1]*dt + 0.5*new_state[2]*dt*dt;
		cout<<"Vehicle ahead id, distance from ego "<<vehicle_ahead_id<<", "<<sensor_fusion[vehicle_ahead_id][5] - ego_state[0]<<endl;
	}

	else
	{
		new_state[2] = MAX_ACCELERATION;
		new_state[1] = min(MAX_SPEED, new_state[2]*dt);
		new_state[0]+= new_state[1]*dt + 0.5*new_state[2]*dt*dt;
		cout<<"No vehicle ahead"<<endl;
	}
	new_state[3] = ego_state[3];
	trajectory.push_back(new_state);

	return trajectory;
}