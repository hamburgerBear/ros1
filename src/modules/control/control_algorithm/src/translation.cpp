#include "control_algorithm/translation.h"

namespace control {

Translation::Translation(const std::string& name,
                         const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Translation::~Translation() {}

void Translation::setGoal(std::shared_ptr<Args> args) {
  args_ = args;
  start_pose_ = toEigen(*injector()->odom_);
  last_pose_ = start_pose_;
  start_time_ = ros::Time::now();
  accumulated_distance_ = 0.0;
  timeout_threshold_ = (targetDistance() / minVelX()) * 1.5;

  ROS_INFO(
      "Control(%s) start_pose(%f, %f, %f) desired_distance(%f) "
      "timeout_threshold(%f)",
      name().c_str(), start_pose_.x(), start_pose_.y(), toDeg(start_pose_.z()),
      targetDistance(), timeout_threshold_);
}

void Translation::update() {
  double v = targetSpeed() * sign(targetDistance());
  double w = 0.0;
  injector()->cmd_vel_.linear.x = v;
  injector()->cmd_vel_.angular.z = w;

  Eigen::Vector3d current_pose = toEigen(*injector()->odom_);
  double distance = hypot(current_pose.x() - last_pose_.x(),
                          current_pose.y() - last_pose_.y());
  if (distance > maxVelX()) {
    ROS_WARN("Control(%s) jump distance(%f) from(%f, %f) to(%f, %f)",
             name().c_str(), distance, last_pose_.x(), last_pose_.y(),
             current_pose.x(), current_pose.y());
    distance = 0.0;
  }

  accumulated_distance_ += distance;
  last_pose_ = current_pose;

  double dt = (ros::Time::now() - start_time_).toSec() * 1000.0;
  ROS_INFO_THROTTLE(
      10.0,
      "Control(%s) cmd_vel(%f, %f) accumulation_distance(%f) cost_time(%f)ms",
      name().c_str(), v, w, accumulated_distance_, dt);
  if (dt > 10.0) {
    ROS_WARN("Control(%s) cost_time(%f)", name().c_str(), dt);
  }
}

}  // namespace control