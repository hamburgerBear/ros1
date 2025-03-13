#include "control_algorithm/rotation.h"

namespace control {

Rotation::Rotation(const std::string& name,
                   const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Rotation::~Rotation() {}

void Rotation::setGoal(std::shared_ptr<Args> args) {
  args_ = std::dynamic_pointer_cast<RotationArgs>(args);
  accumulated_distance_ = 0.0;
  start_pose_ = toEigen(*injector()->odom_);
  last_pose_ = start_pose_;
  start_time_ = ros::Time::now();
  args_->timeout_threshold =
      (desiredDistance() / args_->min_angular_velocity) * 1.5;

  ROS_INFO(
      "Control(%s) start_pose(%f, %f, %f) desired_distance(%f) "
      "timeout_threshold(%f)",
      name().c_str(), start_pose_.x(), start_pose_.y(), toDeg(start_pose_.z()),
      toDeg(desired_distance_), args_->timeout_threshold);
}

void Rotation::update() {
  double v = 0.0;
  double w = args_->max_angular_velocity * sign(desired_distance_);
  injector()->cmd_vel_.linear.x = v;
  injector()->cmd_vel_.angular.z = w;

  Eigen::Vector3d current_pose = toEigen(*injector()->odom_);
  accumulated_distance_ +=
      fabs(shortestAngularDistance(last_pose_.z(), current_pose.z()));

  double dt = (ros::Time::now() - start_time_).toSec() * 1000.0;
  ROS_INFO_THROTTLE(
      10.0,
      "Control(%s) cmd_vel(%f, %f) accumulation_distance(%f) cost_time(%f)ms",
      name().c_str(), v, w, accumulated_distance_, dt);
  if (dt > 10.0) {
    ROS_WARN("Control(%s) cost_time(%f)", name().c_str(), dt);
  }
}

bool Rotation::isFinish() {
  if (accumulated_distance_ >= fabs(desired_distance_)) {
    ROS_INFO(
        "Control(%s) finish, desired_distance(%f) accumulation_distance(%f)",
        name().c_str(), toDeg(desired_distance_), toDeg(accumulated_distance_));
    return true;
  } else
    return false;
}

bool Rotation::isFail() {
  double dt = (ros::Time::now() - start_time_).toSec();
  if (dt >= args_->timeout_threshold) {
    ROS_INFO("Control(%s) fail, timeout_threshold(%f) cost_time(%f)",
             name().c_str(), args_->timeout_threshold, dt);
    return true;
  } else {
    return false;
  }
}

}  // namespace control