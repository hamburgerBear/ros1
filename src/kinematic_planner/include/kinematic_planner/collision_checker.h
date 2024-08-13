// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <memory>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "costmap_2d/costmap_2d_ros.h"

namespace kinematic_planner {

const float UNKNOWN = 255.0;
const float OCCUPIED = 254.0;
const float INSCRIBED = 253.0;
const float MAX_NON_OBSTACLE = 252.0;
const float FREE = 0;
typedef std::vector<geometry_msgs::Point> Footprint;

/** An iterator implementing Bresenham Ray-Tracing. */
class LineIterator {
 public:
  LineIterator(int x0, int y0, int x1, int y1)
      : x0_(x0),
        y0_(y0),
        x1_(x1),
        y1_(y1),
        x_(x0),  // X and Y start of at first endpoint.
        y_(y0),
        deltax_(abs(x1 - x0)),
        deltay_(abs(y1 - y0)),
        curpixel_(0) {
    if (x1_ >= x0_) {  // The x-values are increasing
      xinc1_ = 1;
      xinc2_ = 1;
    } else {  // The x-values are decreasing
      xinc1_ = -1;
      xinc2_ = -1;
    }

    if (y1_ >= y0_) {  // The y-values are increasing
      yinc1_ = 1;
      yinc2_ = 1;
    } else {  // The y-values are decreasing
      yinc1_ = -1;
      yinc2_ = -1;
    }

    if (deltax_ >=
        deltay_) {  // There is at least one x-value for every y-value
      xinc1_ = 0;   // Don't change the x when numerator >= denominator
      yinc2_ = 0;   // Don't change the y for every iteration
      den_ = deltax_;
      num_ = deltax_ / 2;
      numadd_ = deltay_;
      numpixels_ = deltax_;  // There are more x-values than y-values
    } else {                 // There is at least one y-value for every x-value
      xinc2_ = 0;            // Don't change the x for every iteration
      yinc1_ = 0;            // Don't change the y when numerator >= denominator
      den_ = deltay_;
      num_ = deltay_ / 2;
      numadd_ = deltax_;
      numpixels_ = deltay_;  // There are more y-values than x-values
    }
  }

  bool isValid() const { return curpixel_ <= numpixels_; }

  void advance() {
    num_ += numadd_;     // Increase the numerator by the top of the fraction
    if (num_ >= den_) {  // Check if numerator >= denominator
      num_ -= den_;      // Calculate the new numerator value
      x_ += xinc1_;      // Change the x as appropriate
      y_ += yinc1_;      // Change the y as appropriate
    }
    x_ += xinc2_;  // Change the x as appropriate
    y_ += yinc2_;  // Change the y as appropriate

    curpixel_++;
  }

  int getX() const { return x_; }
  int getY() const { return y_; }

  int getX0() const { return x0_; }
  int getY0() const { return y0_; }

  int getX1() const { return x1_; }
  int getY1() const { return y1_; }

 private:
  int x0_;  ///< X coordinate of first end point.
  int y0_;  ///< Y coordinate of first end point.
  int x1_;  ///< X coordinate of second end point.
  int y1_;  ///< Y coordinate of second end point.

  int x_;  ///< X coordinate of current point.
  int y_;  ///< Y coordinate of current point.

  int deltax_;  ///< Difference between Xs of endpoints.
  int deltay_;  ///< Difference between Ys of endpoints.

  int curpixel_;  ///< index of current point in line loop.

  int xinc1_, xinc2_, yinc1_, yinc2_;
  int den_, num_, numadd_, numpixels_;
};

/**
 * @class FootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */
template <typename CostmapT>
class FootprintCollisionChecker {
 public:
  /**
   * @brief A constructor.
   */
  FootprintCollisionChecker();
  /**
   * @brief A constructor.
   */
  explicit FootprintCollisionChecker(CostmapT costmap);
  /**
   * @brief Find the footprint cost in oriented footprint
   */
  double footprintCost(const Footprint& footprint);
  /**
   * @brief Find the footprint cost a a post with an unoriented footprint
   */
  double footprintCostAtPose(double x, double y, double theta,
                             const Footprint& footprint);
  /**
   * @brief Get the cost for a line segment
   */
  double lineCost(int x0, int x1, int y0, int y1) const;
  /**
   * @brief Get the map coordinates from a world point
   */
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
  /**
   * @brief Get the cost of a point
   */
  double pointCost(int x, int y) const;
  /**
   * @brief Set the current costmap object to use for collision detection
   */
  void setCostmap(CostmapT costmap);
  /**
   * @brief Get the current costmap object
   */
  CostmapT getCostmap() { return costmap_; }

 protected:
  CostmapT costmap_;
};

/**
 * @class hybrid_astar_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
    : public FootprintCollisionChecker<costmap_2d::Costmap2D*> {
 public:
  /**
   * @brief A constructor for hybrid_astar_planner::GridCollisionChecker
   * for use when regular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute
   * footprint
   */
  GridCollisionChecker(std::shared_ptr<costmap_2d::Costmap2DROS> costmap,
                       unsigned int num_quantizations);

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius
   * collision checking
   */
  void setFootprint(const /*costmap_2d::*/ Footprint& footprint,
                    const bool& radius, const double& possible_collision_cost);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle bin number of pose to check against (NOT radians)
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(const float& x, const float& y, const float& theta,
                   const bool& traverse_unknown);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(const unsigned int& i, const bool& traverse_unknown);

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost();

  /**
   * @brief Get the angles of the precomputed footprint orientations
   * @return the ordered vector of angles corresponding to footprints
   */
  std::vector<float>& getPrecomputedAngles() { return angles_; }

  /**
   * @brief Get costmap ros object for inflation layer params
   * @return Costmap ros
   */
  std::shared_ptr<costmap_2d::Costmap2DROS> getCostmapROS() {
    return costmap_ros_;
  }

 private:
  /**
   * @brief Check if value outside the range
   * @param min Minimum value of the range
   * @param max Maximum value of the range
   * @param value the value to check if it is within the range
   * @return boolean if in range or not
   */
  bool outsideRange(const unsigned int& max, const float& value);

 protected:
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  std::vector<Footprint> oriented_footprints_;
  Footprint unoriented_footprint_;
  float footprint_cost_;
  bool footprint_is_radius_;
  std::vector<float> angles_;
  float possible_collision_cost_{-1};
};

}  // namespace kinematic_planner

#endif  // COLLISION_CHECKER_H
