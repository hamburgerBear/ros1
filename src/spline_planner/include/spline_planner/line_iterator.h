#ifndef LINE_ITERATOR_H
#define LINE_ITERATOR_H

#include <geometry_msgs/PoseStamped.h>

namespace spline_planner {

class LineIterator {
 public:
  explicit LineIterator();
  ~LineIterator();

  std::vector<geometry_msgs::PoseStamped> generate(
      const std::vector<geometry_msgs::PoseStamped>& in, double resolution = 0.05);
};

}  // namespace spline_planner

#endif
