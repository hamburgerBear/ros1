#include "follow_wall_controller/follow_wall_controller.h"
namespace control {
Transition StateRoot::GetTransition() {
  return InnerEntryTransition<StateFollowWall>();
}

}  // namespace control