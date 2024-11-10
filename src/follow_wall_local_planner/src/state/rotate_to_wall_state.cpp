#include <follow_wall_local_planner/state/rotate_to_wall_state.h>
#include <follow_wall_local_planner/state/data_manager.h>

namespace follow_wall_local_planner
{
RotateToWallState::RotateToWallState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr)
: StateBase(status, data_manager_ptr)
{
}

State RotateToWallState::getCurrentState() const { return State::ROTATE_TO_WALL; }

void RotateToWallState::entry()
{
  init_pose_ = data_manager_ptr_->getGlobalPose();
  ROS_INFO("RotateToWallState::entry");
}

void RotateToWallState::update(geometry_msgs::Twist& cmd_vel)
{
  ROS_INFO("RotateToWallState::update");

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.3;
}

State RotateToWallState::getNextState() const
{
  if(isFinish()) {
    return State::FOLLOW_WALL; 
  } 
  
  return State::ROTATE_TO_WALL;
}

bool RotateToWallState::isFinish() const 
{
  auto current_pose = data_manager_ptr_->getGlobalPose();
  double diff_angle = angles::shortest_angular_distance(tf2::getYaw(current_pose.pose.orientation), 
                                                        tf2::getYaw(init_pose_.pose.orientation));
  ROS_WARN("RotateToWallState diff_angle = %f", diff_angle); 
  if(fabs(diff_angle) >= data_manager_ptr_->getRotateAngle()) 
    return true;
  else 
    return false;
}

}  // namespace follow_wall_local_planner