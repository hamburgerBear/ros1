#include "spline_planner/quadratic_bezier.h"

namespace spline_planner {

QuadraticBezier::QuadraticBezier() {}

QuadraticBezier::~QuadraticBezier() {}

std::vector<geometry_msgs::PoseStamped> QuadraticBezier::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 3) return std::move(out);

  geometry_msgs::PoseStamped p0 = in[0], p1 = in[1], p2 = in[2];
  
  double d1 = hypot(p1.pose.position.x - p0.pose.position.x, 
                    p1.pose.position.y - p0.pose.position.y);

  double d2 = hypot(p2.pose.position.x - p1.pose.position.x, 
                    p2.pose.position.y - p1.pose.position.y);

  double dist = d1 + d2;
  int iterator_num = std::floor(dist / resolution);
  std::vector<double> time(1, 0.0f);
  for(int i = 0; i < iterator_num; i++) 
      time.emplace_back(1.0 / (iterator_num + 1) * (i + 1)) ;
  
  time.emplace_back(1.0f);
  
  geometry_msgs::PoseStamped pose = p2;
  for(auto t : time) {
    pose.pose.position.x = pow(1-t,2)*p0.pose.position.x + \
                           2*t*(1-t)*p1.pose.position.x + \
                           (pow(t,2))*p2.pose.position.x;

    pose.pose.position.y = pow(1-t,2)*p0.pose.position.y + \
                           2*t*(1-t)*p1.pose.position.y + \
                           (pow(t,2))*p2.pose.position.y;
    out.push_back(pose);
  }

  return std::move(out);
}

}  // namespace spline_planner
