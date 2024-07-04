/*********************************************************************
 * Author: Gaojie
 *********************************************************************/
#ifndef SPLINE_PLANNER_DUBINS_H
#define SPLINE_PLANNER_DUBINS_H

#include <vector>

#include <geometry_msgs/PoseStamped.h>

namespace spline_planner {

class Dubins {
 public:
  explicit Dubins();
  ~Dubins();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in,
      double resolution = 0.05, double radius = 0.5);
};

};  // namespace spline_planner

#endif
