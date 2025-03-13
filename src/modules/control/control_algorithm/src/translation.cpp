#include "control_algorithm/translation.h"

namespace control {

Translation::Translation(const std::string& name,
                         const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Translation::~Translation() {}

void Translation::setGoal(std::shared_ptr<Args> args) {
  args_ = std::dynamic_pointer_cast<TranslationArgs>(args);
  accumulated_distance_ = 0.0;
  start_pose_ = toEigen(*injector()->odom_);
  last_pose_ = start_pose_;
  start_time_ = ros::Time::now();
  args_->timeout_threshold =
      (desiredDistance() / args_->min_linear_velocity) * 1.5;

  ROS_INFO(
      "Control(%s) start_pose(%f, %f, %f) desired_distance(%f) "
      "timeout_threshold(%f)",
      name().c_str(), start_pose_.x(), start_pose_.y(), toDeg(start_pose_.z()),
      desired_distance_, args_->timeout_threshold);
}

void Translation::update() {
  double v = args_->max_linear_velocity * sign(desiredDistance());
  double w = 0.0;
  injector()->cmd_vel_.linear.x = v;
  injector()->cmd_vel_.angular.z = w;

  Eigen::Vector3d current_pose = toEigen(*injector()->odom_);
  double distance = hypot(current_pose.x() - last_pose_.x(),
                          current_pose.y() - last_pose_.y());
  if (distance > args_->max_linear_velocity) {
    ROS_WARN("Control(%s) jump distance(%f) from(%f, %f) to(%f, %f)", distance,
             last_pose_.x(), last_pose_.y(), current_pose.x(),
             current_pose.y());
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

bool Translation::isFinish() {
  if (accumulated_distance_ >= fabs(desired_distance_)) {
    ROS_INFO(
        "Control(%s) finish, desired_distance_(%f) accumulation_distance(%f)",
        name().c_str(), desired_distance_, accumulated_distance_);
    return true;
  } else
    return false;
}

bool Translation::isFail() {
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