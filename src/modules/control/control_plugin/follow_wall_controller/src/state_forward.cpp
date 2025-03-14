#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

void StateForward::OnEnter(const double& forward_distance) {
  ROS_INFO("StateForward::OnEnter");
  control_ = std::make_unique<Translation>("translation", Owner().injector_);

  std::vector<Eigen::Vector3d> path;
  path.emplace_back(forward_distance, 0.0, 0.0);
  std::shared_ptr<Translation::TranslationArgs> args =
      std::make_shared<Translation::TranslationArgs>(path, 0.1);
  control_->setGoal(args);
}

void StateForward::Update() { control_->update(); }

void StateForward::OnExit() {}

Transition StateForward::GetTransition() {
  if (control_->isFinish() || control_->isFail()) {
    if (Owner().loseFollowObject(50.0, 85.0))
      return SiblingTransition<StateApproachWall>();
    else
      return SiblingTransition<StateFollowWall>();
  } else
    return NoTransition();
}

}  // namespace control
