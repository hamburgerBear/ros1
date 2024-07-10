#include "spline_planner/line_iterator.h"

namespace spline_planner {

LineIterator::LineIterator() {}

LineIterator::~LineIterator() {}

std::vector<geometry_msgs::PoseStamped> LineIterator::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 2) return std::move(out);

  geometry_msgs::PoseStamped start = in[0], goal = in[1];
  double dist = hypot(goal.pose.position.x - start.pose.position.x, 
                      goal.pose.position.y - start.pose.position.y);
  int iterator_num = std::floor(dist/resolution);
  std::vector<double> time(1, 0.0f);
  for(int i = 0; i < iterator_num; i++) 
      time.emplace_back(1.0 / (iterator_num + 1) * (i + 1)) ;
  
  time.emplace_back(1.0f);
  geometry_msgs::PoseStamped pose = goal;
  for(auto t : time) {
    pose.pose.position.x = (1-t)*start.pose.position.x + t*goal.pose.position.x;
    pose.pose.position.y = (1-t)*start.pose.position.y + t*goal.pose.position.y;
    out.push_back(pose);
  }

  return std::move(out);
}

}  // namespace spline_planner
