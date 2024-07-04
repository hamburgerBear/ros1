#include "spline_planner/dubins.h"

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

namespace spline_planner {

Dubins::Dubins() {}

Dubins::~Dubins() {}

std::vector<geometry_msgs::PoseStamped> Dubins::generate(
    const std::vector<geometry_msgs::PoseStamped>& in, double resolution,
    double radius) {
  std::vector<geometry_msgs::PoseStamped> out;
  if (in.size() != 2) return std::move(out);

  geometry_msgs::PoseStamped start = in[0], goal = in[1];
  ompl::base::StateSpacePtr state_space =
      std::make_unique<ompl::base::DubinsStateSpace>(radius);
  ompl::base::ScopedState<> from(state_space), to(state_space),
      waypoint(state_space);

  from[0] = start.pose.position.x;
  from[1] = start.pose.position.y;
  from[2] = tf2::getYaw(start.pose.orientation);
  to[0] = goal.pose.position.x;
  to[1] = goal.pose.position.y;
  to[2] = tf2::getYaw(goal.pose.orientation);
  float distance = state_space->distance(from(), to());

  unsigned int num_pts = std::ceil(distance / resolution);
  geometry_msgs::PoseStamped pose = goal;
  for (unsigned int i = 0; i <= num_pts; ++i) {
    state_space->interpolate(from(), to(), (float)i / num_pts, waypoint());
    auto reals = waypoint.reals();
    pose.pose.position.x = (float)reals[0];
    pose.pose.position.y = (float)reals[1];
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    pose.pose.orientation = tf2::toMsg(quaternion);
    out.push_back(pose);
  }

  return std::move(out);
}

};  // namespace spline_planner
