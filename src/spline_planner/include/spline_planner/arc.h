#ifndef ARC_H
#define ARC_H

#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>

namespace spline_planner {

class Arc {
 public:
  explicit Arc();
  ~Arc();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);
};

}  // namespace spline_planner

#endif
