#ifndef ROUNDED_H
#define ROUNDED_H

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>

namespace spline_planner {

class Rounded {
 public:
  explicit Rounded();
  ~Rounded();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, 
      double resolution = 0.05, double radius = 0.5);
};

}  // namespace spline_planner

#endif
