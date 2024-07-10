#include "spline_planner/rounded.h"
#include "spline_planner/line_iterator.h"
#include "spline_planner/cubic_bezier.h"

namespace spline_planner {

Rounded::Rounded() {}

Rounded::~Rounded() {}

std::vector<geometry_msgs::PoseStamped> Rounded::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution, double radius) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() < 3) return std::move(out);

  auto toEigen2d = [](const geometry_msgs::PoseStamped& pose) -> Eigen::Vector2d {
  	Eigen::Vector2d eigen_2d;
  	eigen_2d[0] = pose.pose.position.x;
  	eigen_2d[1] = pose.pose.position.y;
  	return eigen_2d;
  };

  auto toPoseStamped = [=](const Eigen::Vector2d& pose) -> geometry_msgs::PoseStamped {
  	geometry_msgs::PoseStamped pose_stamped = in.back();
  	pose_stamped.pose.position.x = pose.x();
  	pose_stamped.pose.position.y = pose.y();
  	return pose_stamped;
  };

	std::vector<geometry_msgs::PoseStamped> path_tmp;  	
	Eigen::Vector2d curve_segment_start = toEigen2d(in.front());
	double f13 = 1.0 / 3;
	double f23 = 2.0 / 3;
	Eigen::Vector2d targetPoint = toEigen2d(in.back());
  
  for (int i = 1; i < (in.size() - 1); i++) {
		Eigen::Vector2d curr = toEigen2d(in[i]);
		Eigen::Vector2d prev = toEigen2d(in[i - 1]);
  	Eigen::Vector2d next = toEigen2d(in[i + 1]);
  	double prevDistance = (curr-prev).norm() / 2;
    double nextDistance = (curr-next).norm() / 2;
    const double startMove = std::min(radius, prevDistance);
  	const double endMove = std::min(radius, nextDistance);
  	double angle = atan2(prev(1)-curr(1), prev(0)-curr(0));
  	Eigen::Vector2d roundedStart{curr(0) + cos(angle) * startMove, curr(1) + sin(angle) * startMove};
  	angle = atan2(next(1)-curr(1), next(0)-curr(0));
    Eigen::Vector2d roundedEnd{curr(0) + cos(angle) * endMove, curr(1) + sin(angle) * endMove};
   	Eigen::Vector2d control1{f13 * roundedStart(0) + f23 * curr(0), f23 * curr(1) + f13 * roundedStart(1)};
		Eigen::Vector2d control2{f13 * roundedEnd(0) + f23 * curr(0), f23 * curr(1) + f13 * roundedEnd(1)};
		
		path_tmp.clear();
		path_tmp = {toPoseStamped(curve_segment_start), toPoseStamped(roundedStart)};
		LineIterator line_iterator;
		path_tmp = line_iterator.generate(path_tmp, resolution);
		out.insert(out.end(), path_tmp.begin(), path_tmp.end());
		out.pop_back();

		path_tmp.clear();
		path_tmp = {toPoseStamped(roundedStart), toPoseStamped(control1), 
							  toPoseStamped(control2), toPoseStamped(roundedEnd)};
		CubicBezier cubic_bezier;
		path_tmp = cubic_bezier.generate(path_tmp, resolution);
		out.insert(out.end(), path_tmp.begin(), path_tmp.end());
		out.pop_back();
		curve_segment_start = roundedEnd;
  }

  path_tmp.clear();
	path_tmp = {toPoseStamped(curve_segment_start), toPoseStamped(targetPoint)};
	LineIterator line_iterator;
	path_tmp = line_iterator.generate(path_tmp, resolution);
	out.insert(out.end(), path_tmp.begin(), path_tmp.end());

  return std::move(out);
}

}  // namespace spline_planner
