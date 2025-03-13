#include "follow_wall_controller/follow_wall_controller.h"

namespace control {

void StateForward::OnEnter(const double& forward_distance) {
  std::shared_ptr<Translation::TranslationArgs> args =
      std::make_shared<Translation::TranslationArgs>();

  if (forward_distance > 0.0) {
    control_->desired_distance_ = forward_distance;
  } else {
    // args->distance = Owner().params().forwardDistance();
  }

  args->max_linear_velocity = Owner().params()->ForwardLinearVelMax();
  args->min_linear_velocity = Owner().params()->ForwardLinearVelMin();
  // args->max_angular_velocity = Owner().params().ForwardLinearVelMin();
  args->acc_linear_velocity = Owner().params()->ForwardLinearAcc();
  args->dcc_linear_velocity = Owner().params()->ForwardLinearDcc();
  args->acc_angular_velocity = Owner().params()->ForwardLinearDcc();
  args->fixed_head = true;

  control_ = std::make_unique<Translation>("translation", Owner().injector_);
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
