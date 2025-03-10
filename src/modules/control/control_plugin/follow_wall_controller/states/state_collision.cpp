// #include "follow_wall_controller/states/state_collision.h"

// namespace control {

// void StateCollision::OnEnter(/*后退距离*/) {
//   //判断是否需要后退，软碰撞无需后退
//   which_control_ = "backward";
//   backward_ = std::make_unique<Translation>("backward", Owner()->injector_);
//   rotation_ = std::make_unique<Rotation>("rotation", Owner()->injector_);
// }

// void StateCollision::Update() {
//   if (which_control_ == "backward") {
//     backward_->update();
//     if (backward_->isFinish() || backward_->isFail()) {
//       which_control_ = "rotation";  //可以重新计算合适的旋转角度
//     }
//   } else
//     rotation_->update();
// }

// void StateCollision::OnExit() {}

// Transition StateCollision::GetTransition() {
//   if (which_control_ == "rotation" && control_->isFinish() ||
//       control_->isFail()) {
//     if (Owner->loseObject())
//       return SiblingTransition<StateArc>();
//     else
//       return SiblingTransition<StateFollowWall>();
//   } else
//     return NoTransition();
// }

// }  // namespace control
