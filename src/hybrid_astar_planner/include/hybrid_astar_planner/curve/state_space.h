#ifndef STATE_SPACE_H
#define STATE_SPACE_H

// namespace nav2_smac_planner {

#include <limits>

class StateSpace
{
public:
	// StateSpace();
	// ~StateSpace();

	virtual void init(double q0[3], double q1[3]) = 0;
  	virtual double distance() = 0;
  	virtual void interpolate(double q0[3], double q1[3], double seg, double s[3]) = 0;
};

// }  // namespace nav2_smac_planner

#endif
