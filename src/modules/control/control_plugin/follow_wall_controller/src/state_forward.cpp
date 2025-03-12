// #include "follow_wall_controller/states/state_approach.h"

// namespace control {

// void StateForward::OnEnter() {
//   control_ = std::make_unique<Arc>("arc", Owner()->injector_);
//   control_->setGoal(Owner()->arc_args_);
// }

// void StateForward::Update() { control_->update(); }

// void StateForward::OnExit() {}

// Transition StateForward::GetTransition() {
//   if (control_->isFinish() || control_->isFail()) {
//     if (Owner->loseObject())
//       return SiblingTransition<StateArc>();
//     else
//       return SiblingTransition<StateFollowWall>();
//   } else
//     return NoTransition();
// }

// }  // namespace control
