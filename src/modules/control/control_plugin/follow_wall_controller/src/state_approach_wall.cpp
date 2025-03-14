#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

/*
  1. 从Ownner中获取传感器资源，以及一些针对该控制器的(Root)通用算法
  2. 从OnExit传入任务目标(Action goal)
*/
void StateApproachWall::OnEnter() {
  ROS_INFO("StateApproachWall::OnEnter");
  control_ = std::make_unique<Arc>("arc", Owner().injector_);

  std::vector<Eigen::Vector3d> path;
  path.emplace_back(toRad(80.0), 0.0, 0.0);
  std::shared_ptr<Arc::ArcArgs> args =
      std::make_shared<Arc::ArcArgs>(path, 0.1, -0.15);
  control_->setGoal(args);
}

void StateApproachWall::Update() { control_->update(); }

void StateApproachWall::OnExit() {}

Transition StateApproachWall::GetTransition() {
  if (control_->isFinish() || control_->isFail())
    return SiblingTransition<StateForward>(0.02);
  else if (isFrontSideApproach())
    return SiblingTransition<StateFollowWall>();
  else if (isSideApproach())
    return SiblingTransition<StateForward>(0.02);
  else
    return NoTransition();
}

bool StateApproachWall::isFrontSideApproach() const {
  double dir = Owner().getFollowDir();
  double lateral_distance =
      Owner().getLateralDistanceFromScan(50.0 * dir, 75.0 * dir);

  if (lateral_distance <=
      Owner().params()->RobotRadius() + Owner().params()->FollowWallDistance())
    return true;
  else
    return false;
}

bool StateApproachWall::isSideApproach() const {
  double dir = Owner().getFollowDir();
  double lateral_distance =
      Owner().getLateralDistanceFromScan(75.0 * dir, 88.0 * dir);

  if (lateral_distance <=
      Owner().params()->RobotRadius() + Owner().params()->FollowWallDistance())
    return true;
  else
    return false;
}

}  // namespace control
