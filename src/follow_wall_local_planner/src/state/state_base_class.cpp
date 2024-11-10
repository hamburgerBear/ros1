#include "follow_wall_local_planner/state/state_base_class.h"

namespace follow_wall_local_planner {

std::ostream & operator<<(std::ostream & ostream, const State & state)
{
  switch (state) {
    case State::NO_STATE:
      ostream << std::string("NO_STATE");
      break;
    case State::FOLLOW_WALL:
      ostream << std::string("FOLLOW_WALL");
      break;
    // case State::FOLLOWING_LANE:
    //   ostream << std::string("FOLLOWING_LANE");
    //   break;
    // case State::EXECUTING_LANE_CHANGE:
    //   ostream << std::string("EXECUTING_LANE_CHANGE");
    //   break;
    // case State::ABORTING_LANE_CHANGE:
    //   ostream << std::string("ABORTING_LANE_CHANGE");
    //   break;
    // case State::FORCING_LANE_CHANGE:
    //   ostream << std::string("FORCING_LANE_CHANGE");
    //   break;
    // case State::BLOCKED_BY_OBSTACLE:
    //   ostream << std::string("BLOCKED_BY_OBSTACLE");
    //   break;
    // default:
    //   ostream << std::string("NO_STATE");
    //   break;
  }
  return ostream;
}

StateBase::StateBase(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr/*,
  const std::shared_ptr<RouteHandler> & route_handler_ptr*/)
: data_manager_ptr_(data_manager_ptr), /*route_handler_ptr_(route_handler_ptr),*/ status_(status)
{
}

Status StateBase::getStatus() const { return status_; }

} // namespace follow_wall_local_planner
