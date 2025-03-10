// #include "follow_wall_controller/states/state_approach.h"

// namespace control {

// /*
//   1. 从Ownner中获取传感器资源，以及一些针对该控制器的(Root)通用算法
//   2. 从OnExit传入任务目标(Action goal)
// */
// void StateApproach::OnEnter() {
//   control_ = std::make_unique<Arc>("arc", Owner()->injector_);
//   control_->setGoal(Owner()->arc_args_);
// }

// void StateApproach::Update() { control_->update(); }

// void StateApproach::OnExit() {}

// //超时
// Transition StateApproach::GetTransition() {
//   if (control_->isFinish() || control_->isFail())
//     return SiblingTransition<StateForward>();
//   else if (isFrontSideApproach())
//     return SiblingTransition<StateFollowWall>();
//   else if (isSideApproach())
//     return SiblingTransition<StateForward>();
//   else
//     return NoTransition();
// }

// bool StateApproach::isFrontSideApproach() { return false; }

// bool StateApproach::isSideApproach() { return false; }

// }  // namespace control
