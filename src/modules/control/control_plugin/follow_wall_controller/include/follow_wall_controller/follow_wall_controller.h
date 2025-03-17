#pragma once

#include "control_algorithm/arc.h"
#include "control_algorithm/rotation.h"
#include "control_algorithm/translation.h"
#include "control_common/control_base.h"
#include "control_common/hsm.h"
#include "control_common/utils.h"
#include "follow_wall_cfg.h"
#include "follow_wall_visual.h"

using namespace hsm;
namespace control {

class FollowWallController : public PluginBase {
 public:
  FollowWallController() = default;
  ~FollowWallController() = default;

  bool init(const std::string& name,
            const DependencyInjector::Ptr& injector) override;
  PluginStage run() override;

 public:
  double getFollowDir() const;
  double getLateralDistanceFromScan(const double& angle1,
                                    const double& angle2) const;
  bool loseFollowObject(const double& angle1, const double& angle2) const;

  boost::shared_ptr<Params> params() const;
  boost::shared_ptr<Visualization> visual() const;

 private:
  std::shared_ptr<hsm::StateMachine> state_machine_;

  ros::NodeHandle nh_;
  Eigen::Isometry3d base_to_laser_;
  PointCloud pointcloud_;                   //激光点云
  DiscretePointCloud discrete_pointcloud_;  //离散的激光点云
  boost::shared_ptr<Params> params_;
  boost::shared_ptr<Visualization> visualization_;
  double follow_dir_;
  //雷达数据去畸变、雷达数据坐标系变换、雷达数据稀疏化
};

struct StateRoot : StateWithOwner<FollowWallController> {
  virtual void OnEnter();
  virtual void Update();
  virtual void OnExit();
  virtual Transition GetTransition();
};

struct StateFollowWall : StateWithOwner<FollowWallController> {
  virtual void OnEnter();
  virtual void Update();
  virtual void OnExit();
  virtual Transition GetTransition();

 private:
};

struct StateApproachWall : StateWithOwner<FollowWallController> {
  virtual void OnEnter();
  virtual void Update();
  virtual void OnExit();
  virtual Transition GetTransition();

 private:
  bool isFrontSideApproach() const;
  bool isSideApproach() const;

  std::unique_ptr<Arc> control_;
};

struct StateCollision : StateWithOwner<FollowWallController> {
  virtual void OnEnter();
  virtual void Update();
  virtual void OnExit();
  virtual Transition GetTransition();

 private:
  double calcRatationTheta();

  std::unique_ptr<Translation> backward_;
  std::unique_ptr<Rotation> rotation_;
  std::string which_control_;
};

struct StateForward : StateWithOwner<FollowWallController> {
  virtual void OnEnter(const double& forward_distance = -1.0);
  virtual void Update();
  virtual void OnExit();
  virtual Transition GetTransition();

 private:
  std::unique_ptr<Translation> control_;
};

struct StateFollowBoundary : StateWithOwner<FollowWallController> {
  virtual void OnEnter();
  virtual void Update();
  virtual void OnExit();
  virtual Transition GetTransition();

 private:
  // std::unique_ptr<Translation> control_;
};

}  // namespace control