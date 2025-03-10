#pragma once

#include "control_common/hsm.h"

namespace control {

using namespace hsm;
class FollowWallController;

struct StateRoot : StateWithOwner<FollowWallController> {
  virtual Transition GetTransition();
};

}  // namespace control
