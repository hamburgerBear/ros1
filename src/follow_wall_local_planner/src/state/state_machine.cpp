#include <follow_wall_local_planner/state/follow_wall_state.h>
#include <follow_wall_local_planner/state/rotate_to_wall_state.h>
#include <follow_wall_local_planner/state/arc_to_wall_state.h>
// #include <follow_wall_local_planner/state/blocked_by_obstacle.h>
// #include <follow_wall_local_planner/state/executing_lane_change.h>
// #include <follow_wall_local_planner/state/following_lane.h>
// #include <follow_wall_local_planner/state/forcing_lane_change.h>
#include <follow_wall_local_planner/state/state_machine.h>
// #include <ros/ros.h>

#include <visualization_msgs/Marker.h>

namespace follow_wall_local_planner
{
StateMachine::StateMachine(
  const std::shared_ptr<DataManager> & data_manager_ptr/*,
  const std::shared_ptr<RouteHandler> & route_handler_ptr*/)
  : data_manager_ptr_(data_manager_ptr)/*, route_handler_ptr_(route_handler_ptr)*/
{
}

void StateMachine::init()
{
  Status empty_status;
  state_obj_ptr_ =
    std::make_unique<FollowWallState>(empty_status, data_manager_ptr_/*, route_handler_ptr_*/);
  state_obj_ptr_->entry();
}

// void StateMachine::init(const autoware_planning_msgs::Route & route) { init(); }

void StateMachine::updateState(geometry_msgs::Twist& cmd_vel)
{
  // update state status
  state_obj_ptr_->update(cmd_vel);
  State current_state = state_obj_ptr_->getCurrentState();
  State next_state = state_obj_ptr_->getNextState();

  // Transit to next state
  if (next_state != current_state) {
    ROS_INFO_STREAM("changing state: " << current_state << " => " << next_state);
    const auto previous_status = state_obj_ptr_->getStatus();
    switch (next_state) {
      case State::FOLLOW_WALL:
        state_obj_ptr_ = std::make_unique<FollowWallState>(
          previous_status, data_manager_ptr_/*, route_handler_ptr_*/);
        break;
      case State::ROTATE_TO_WALL:
        state_obj_ptr_ = std::make_unique<RotateToWallState>(
          previous_status, data_manager_ptr_);
        break;
      case State::ARC_TO_WALL:
        state_obj_ptr_ = std::make_unique<ArcToWallState>(
          previous_status, data_manager_ptr_);
        break;
    }
    state_obj_ptr_->entry();
    state_obj_ptr_->update(cmd_vel);
  }
}


Status StateMachine::getStatus() const { return state_obj_ptr_->getStatus(); }

}  // namespace follow_wall_local_planner