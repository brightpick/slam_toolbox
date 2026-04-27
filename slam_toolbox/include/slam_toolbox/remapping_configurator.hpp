/*
 * Copyright (c) 2026, Brightpick
 *
 * ROS-parameter adapter for the remapping feature:
 *   - reads `remapping_polygon` / `remapping_polygon_units` ROS params,
 *   - queues pixel-coord polygons until the map's grid dimensions are known,
 *   - calls SessionState::setRemapping() when a polygon is ready.
 *
 * The predicate hooks (fixed-pose pinning during optimisation, loop-closure
 * candidate filtering) are factory methods on SessionState; SlamToolbox
 * wires them once at startup against the live state.
 */

#ifndef SLAM_TOOLBOX_REMAPPING_CONFIGURATOR_H_
#define SLAM_TOOLBOX_REMAPPING_CONFIGURATOR_H_

#include <optional>
#include <vector>

#include <ros/ros.h>
#include <karto_sdk/Karto.h>

#include "slam_toolbox/slam_mapper.hpp"

namespace slam_toolbox
{

class RemappingConfigurator
{
public:
  // Read `remapping_polygon` + `remapping_polygon_units` from `nh`.
  //   units="world"  → smapper is configured immediately.
  //   units="pixels" → polygon is queued until resolvePendingPolygon() runs
  //                    with the loaded grid dimensions.
  // Invalid / missing params are a no-op.  Errors are logged and ignored.
  void loadFromRosParams(ros::NodeHandle& nh, mapper_utils::SMapper& smapper);

  // Resolve a queued pixel-coord polygon to world coords using the grid
  // dimensions computed from `smapper`'s currently-loaded scans at
  // `resolution`, then configure smapper.  No-op when no polygon is pending.
  void resolvePendingPolygon(mapper_utils::SMapper& smapper, double resolution);

private:
  std::optional<Polygon> pending_pixel_polygon_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_REMAPPING_CONFIGURATOR_H_
