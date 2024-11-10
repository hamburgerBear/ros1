#ifndef FOLLOW_WALL_STATE_H
#define FOLLOW_WALL_STATE_H

#include <follow_wall_local_planner/state/state_base_class.h>

// #include <autoware_perception_msgs/DynamicObjectArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>

// #include <lanelet2_core/primitives/Lanelet.h>

namespace follow_wall_local_planner
{
class FollowWallState : public StateBase
{
private:
  // State transition conditions
  bool isLongitudinalApproach() const;
  bool isLateralDeviation() const;
  
public:
  FollowWallState(
    const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr/*,
    const std::shared_ptr<RouteHandler> & route_handler_ptr*/);
  // override virtual functions
  void entry() override;
  void update(geometry_msgs::Twist& cmd_vel) override;
  State getNextState() const override;
  State getCurrentState() const override;
  // autoware_planning_msgs::PathWithLaneId getPath() const override;
};
}  // namespace follow_wall_local_planner

#endif  // FOLLOW_WALL_STATE_H