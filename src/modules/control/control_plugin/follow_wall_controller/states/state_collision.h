// #pragma once

// #include "control_algorithm/translation.h"
// #include "control_common/hsm.h"

// namespace control {

// using namespace hsm;
// class FollowWallController;

// struct StateForward : StateWithOwner<FollowWallController> {
//   virtual void OnEnter();
//   virtual void Update();
//   virtual void OnExit();
//   virtual Transition GetTransition();

//  private:
//   std::unique_ptr<Translation> backward_;
//   std::unique_ptr<Translation> rotation_;
//   std::string which_control_;
// };

// }  // namespace control
