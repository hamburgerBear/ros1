#include <follow_wall_local_planner/state/arc_to_wall_state.h>
#include <follow_wall_local_planner/state/data_manager.h>

namespace follow_wall_local_planner
{
ArcToWallState::ArcToWallState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr)
: StateBase(status, data_manager_ptr)
{
}

State ArcToWallState::getCurrentState() const { return State::ROTATE_TO_WALL; }

void ArcToWallState::entry()
{
  init_pose_ = data_manager_ptr_->getGlobalPose();
  ROS_INFO("ArcToWallState::entry");
}

void ArcToWallState::update(geometry_msgs::Twist& cmd_vel)
{
  ROS_INFO("ArcToWallState::update");

  cmd_vel.linear.x = 0.13;
  cmd_vel.angular.z = -1.0;
}

State ArcToWallState::getNextState() const
{
  if(isFinish()) {
    return State::FOLLOW_WALL; 
  } else if(isLateralApproach()) {
    return State::FOLLOW_WALL; 
  } else 
    return State::ARC_TO_WALL;
} 

bool ArcToWallState::isFinish() const {
  auto current_pose = data_manager_ptr_->getGlobalPose();
  double diff_angle = angles::shortest_angular_distance(tf2::getYaw(current_pose.pose.orientation), 
                                                        tf2::getYaw(init_pose_.pose.orientation));
  ROS_WARN("ArcToWallState diff_angle = %f", diff_angle); 
  if(fabs(diff_angle) >= angles::from_degrees(80.0)) 
    return true;
  else 
    return false;
}

bool ArcToWallState::isLateralApproach() const 
{
  double range = data_manager_ptr_->getLateralDistanceFromScan(angles::from_degrees(-50.0), angles::from_degrees(-85.0));
  double line_laser_range = data_manager_ptr_->getLinearScan(angles::from_degrees(-75.0));
  range = std::min(line_laser_range, range);
  ROS_WARN("ArcToWallState range = %f, target = %f", range, data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance()); 
  if(range <= (data_manager_ptr_->getRobotRadius() + data_manager_ptr_->getFollowWallDistance()))
    return true;
  else 
    return false;
}

}  // namespace follow_wall_local_planner
