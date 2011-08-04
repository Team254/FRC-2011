#include "DiscreteAccelFilter.h"

DiscreteAccelFilter::DiscreteAccelFilter(double currPos, double currVel, double currAcc)
: AccelFilterBase(currPos, currVel, currAcc)
{}

int64_t DiscreteAccelFilter::sd(int64_t v, int64_t a)
{
	if (v < 0) {
		v = -v;
		return -(v + v % a) * (v - v % a + a) / (2 * a);
	} else {
		return (v + v % a) * (v - v % a + a) / (2 * a);
	}
}

// Integer acceleration function.
void DiscreteAccelFilter::CalcSystem_Discrete(int64_t distance_to_target, int64_t v, int64_t goal_v, int64_t max_v, int64_t max_a)
{
	int64_t sign = signum(distance_to_target);
	int64_t gooda;
	// Compute our maximum acceleration
	if (sign * v + max_a > max_v) {
		gooda = sign * max_v - v;
	} else if (labs(v - goal_v) <= max_a && distance_to_target == 0) {
		gooda = goal_v - v;
		sign = signum(gooda);
	} else if (labs(distance_to_target) <= max_a) {
		gooda = distance_to_target - v;
	} else {
		gooda = sign * max_a;
	}
	
	// Loop while accelerating that way would throw us too far
	while (sign * (v + sd(v + gooda, max_a) - sd(goal_v, max_a) - distance_to_target) > 0) {
		gooda = gooda - sign;
	} 
	//convert from mm/timestep^2 to m/s^2
	UpdateVals(gooda/100.0,.01 /*1/100 second timestep*/);
}

// Converts everything to discrete time and calculates the values from there
void DiscreteAccelFilter::CalcSystem(double distance_to_target, double v, double goal_v, double max_a, double max_v, double dt)
{
	// In milli-meters
	uint64_t int_distance = (int)(distance_to_target * 10000);
	
	// In milli-meters per timestep.  Assuming a refresh rate of 100 Hz
	uint64_t int_velocity = (int)(v * 100);
	uint64_t int_goal_v = (int)(goal_v * 100);
	uint64_t int_max_v = (int)(max_v * 100);
	
	uint64_t int_max_a = (int)(max_a);
	
	//printf("velocity %d, goal_v %d, max_v %d, max_a %d, error %d\n", (int)int_velocity, (int)int_goal_v, (int)int_max_v, (int)int_max_a, (int) int_distance);
	//printf("Accel %d\n", get_acceleration (int_distance, int_velocity, int_goal_v, int_max_v, int_max_a));
	
	CalcSystem_Discrete(int_distance, int_velocity, int_goal_v, int_max_v, int_max_a);
}
