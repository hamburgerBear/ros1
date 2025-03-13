#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

void StateRoot::OnEnter() { ROS_INFO("StateRoot::OnEnter"); }

void StateRoot::Update() { ROS_INFO("StateRoot::Update"); }

void StateRoot::OnExit() { ROS_INFO("StateRoot::OnExit"); }

Transition StateRoot::GetTransition() {
  return InnerEntryTransition<StateFollowWall>();
}

}  // namespace control