#include "control_algorithm/arc.h"

namespace control {

Arc::Arc(const std::string& name, const DependencyInjector::Ptr& injector)
    : ControlBase(name, injector) {}

Arc::~Arc() {}

void Arc::setGoal(std::shared_ptr<Args> args) {
  args_ = args;
  start_pose_ = toEigen(*injector()->odom_);
  last_pose_ = start_pose_;
  start_time_ = ros::Time::now();
  accumulated_distance_ = 0.0;
  timeout_threshold_ = (targetDistance() / minVelTheta()) * 1.5;

  ROS_INFO(
      "Control(%s) start_pose(%f, %f, %f) desired_distance(%f) "
      "timeout_threshold(%f)",
      name().c_str(), startPose().x(), startPose().y(), toDeg(startPose().z()),
      toDeg(targetDistance()), timeoutThreshold());
}

void Arc::update() {
  double v = targetSpeed();
  double w = targetSpeed() / (rotationRadius() * sign(targetDistance()));
  injector()->cmd_vel_.linear.x = v;
  injector()->cmd_vel_.angular.z = w;

  Eigen::Vector3d current_pose = toEigen(*injector()->odom_);
  double distance =
      fabs(shortestAngularDistance(lastPose().z(), current_pose.z()));
  if (distance > maxVelTheta()) {
    ROS_WARN("Control(%s) jump distance(%f) from(%f) to(%f)", name().c_str(),
             toDeg(distance), toDeg(lastPose().z()), toDeg(current_pose.z()));
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

double Arc::rotationRadius() const {
  return std::dynamic_pointer_cast<ArcArgs>(args_)->rotation_radius;
}

}  // namespace control
