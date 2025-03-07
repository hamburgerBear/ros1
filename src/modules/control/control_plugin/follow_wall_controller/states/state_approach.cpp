#include "follow_wall_controller/states/state_approach.h"

namespace control {

// 有一些基础函数应放在utils，考虑是放在follow_wall_control的utils里还是再control_common里
Transition StateApproach::transition() {
  if (isFinish())
    return std::make_pair("state_forward", nullptr);
  else if (/*靠近墙，切换followwall*/)
    return std::make_pair("state_follow_wall", nullptr);
  else
    return std::make_pair("state_approach", nullptr);

}  // namespace control