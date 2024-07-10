#include "spline_planner/cubic_bezier.h"

namespace spline_planner {

CubicBezier::CubicBezier() {}

CubicBezier::~CubicBezier() {}

std::vector<geometry_msgs::PoseStamped> CubicBezier::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 4) return std::move(out);

  geometry_msgs::PoseStamped p0 = in[0], p1 = in[1], p2 = in[2], p3 = in[3];
  
  double d1 = hypot(p1.pose.position.x - p0.pose.position.x, 
                    p1.pose.position.y - p0.pose.position.y);

  double d2 = hypot(p2.pose.position.x - p1.pose.position.x, 
                    p2.pose.position.y - p1.pose.position.y);

  double d3 = hypot(p3.pose.position.x - p2.pose.position.x, 
                    p3.pose.position.y - p2.pose.position.y);

  double dist = d1 + d2 + d3;
  int iterator_num = std::floor(dist / resolution);
  std::vector<double> time(1, 0.0f);
  for(int i = 0; i < iterator_num; i++) 
      time.emplace_back(1.0 / (iterator_num + 1) * (i + 1)) ;
  
  time.emplace_back(1.0f);
  
  geometry_msgs::PoseStamped pose = p3;
  for(auto t : time) {
    pose.pose.position.x = p0.pose.position.x*(pow(1-t, 3)) + \
                           3*p1.pose.position.x*t*(pow(1-t, 2)) + \
                           3*p2.pose.position.x*(pow(t, 2))*(1-t) + \
                           p3.pose.position.x*(pow(t, 3));

    pose.pose.position.y = p0.pose.position.y*(pow(1-t, 3)) + \
    					   3*p1.pose.position.y*t*(pow(1-t, 2)) + \
    					   3*p2.pose.position.y*(pow(t, 2))*(1-t) + \
    					   p3.pose.position.y*(pow(t, 3));
    out.push_back(pose);
  }

  return std::move(out);
}

}  // namespace spline_planner
	                               