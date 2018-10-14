#include "behavior_planner.h"
#include <typeinfo>

using namespace std;

int main()
{
	FSM machine;
	Vehicle vehicle(0,1,1,0);//s, s_dot, d, d_dot
	vehicle.vehicles[0] = {6,1,0,1,0};//s, s_dot, s_dot_dot, d, d_dot
	auto ret = machine.next_state(vehicle, "KL");

}