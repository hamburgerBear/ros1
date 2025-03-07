#pragma once

#include "control_algorithm/arc.h"

namespace control {

using struct StateApproach : StateWithOwner<> {
  virtual void OnEnter() {}
  virtual void Update() {}
  virtual void OnExit() {}
  // virtual Transition GetTransition() { return SiblingTransition<Second>(); }

}

class StateApproach : Arc {
  StateApproach() = default;
  ~StateApproach() = default;

  Transition transition() override;
};

}  // namespace control