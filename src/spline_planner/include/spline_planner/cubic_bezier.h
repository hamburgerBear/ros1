#ifndef CUBIC_BEZIER_H
#define CUBIC_BEZIER_H

#include <geometry_msgs/PoseStamped.h>

namespace spline_planner {

class CubicBezier {
 public:
  explicit CubicBezier();
  ~CubicBezier();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);
};

}  // namespace spline_planner

#endif
