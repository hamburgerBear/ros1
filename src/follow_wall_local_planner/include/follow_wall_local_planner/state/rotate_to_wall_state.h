#ifndef ROTATE_TO_WALL_STATE_H
#define ROTATE_TO_WALL_STATE_H

#include <follow_wall_local_planner/state/state_base_class.h>

namespace follow_wall_local_planner
{
class RotateToWallState : public StateBase
{
private:
  // State transition conditions
  bool isFinish() const;
  
  geometry_msgs::PoseStamped init_pose_;

public:
  RotateToWallState(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr);
  // override virtual functions
  void entry() override;
  void update(geometry_msgs::Twist& cmd_vel) override;
  State getNextState() const override;
  State getCurrentState() const override;
};
}  // namespace follow_wall_local_planner

#endif  // ROTATE_TO_WALL_STATE_H
