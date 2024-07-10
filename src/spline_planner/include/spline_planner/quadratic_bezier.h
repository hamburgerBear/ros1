#ifndef QUADRATIC_BEZIER_H
#define QUADRATIC_BEZIER_H

#include <geometry_msgs/PoseStamped.h>

namespace spline_planner {

class QuadraticBezier {
 public:
  explicit QuadraticBezier();
  ~QuadraticBezier();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);
};

}  // namespace spline_planner

#endif
