// #pragma once

// #include "control_algorithm/arc.h"
// #include "control_common/hsm.h"

// namespace control {

// using namespace hsm;
// class FollowWallController;

// struct StateApproach : StateWithOwner<FollowWallController> {
//   virtual void OnEnter();
//   virtual void Update();
//   virtual void OnExit();
//   virtual Transition GetTransition();

//  private:
//   isFrontSideApproach();
//   isSideApproach();

//   std::unique_ptr<Arc> control_;
// };

// }  // namespace control
