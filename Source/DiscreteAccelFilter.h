#ifndef DISCRETE_ACCEL_FILTER_H
#define DISCRETE_ACCEL_FILTER_H

#include "AccelFilterBase.hpp"

/**
 * A discrete time acceleration profile.
 *
 * Discrete data is polled periodically at a given time increment.
 */
class DiscreteAccelFilter : public AccelFilterBase {
public:
	DiscreteAccelFilter(double currPos=0, double currVel=0, double currAcc=0);
	virtual ~DiscreteAccelFilter() {}
	virtual void CalcSystem(double distance_to_target, double v, double goal_v, double max_a, double max_v, double dt /*unused - assumed 100Hz*/);	
private:
	void CalcSystem_Discrete(int64_t distance_to_target, int64_t v, int64_t goal_v, int64_t max_v, int64_t max_a);
	int64_t sd(int64_t v, int64_t a);
};

#endif
