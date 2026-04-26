/*
 * Copyright (c) 2026, Brightpick
 *
 * Wires the remapping feature into the slam_toolbox runtime:
 *   - reads `remapping_polygon` / `remapping_polygon_units` ROS params,
 *   - queues pixel-coord polygons until the map's grid dimensions are known,
 *   - installs the node-fixed predicate on the mapper so non-remap nodes
 *     stay pinned during pose-graph optimisation,
 *   - installs the loop-closure candidate filter so scans superseded by a
 *     later remapping session are skipped.
 *
 * Pulled out of SlamToolbox so the feature lives in one place and the
 * original common.cpp has minimal feature-specific churn.
 */

#ifndef SLAM_TOOLBOX_REMAPPING_CONFIGURATOR_H_
#define SLAM_TOOLBOX_REMAPPING_CONFIGURATOR_H_

#include <optional>
#include <vector>

#include <ros/ros.h>
#include <karto_sdk/Karto.h>
#include <karto_sdk/LoopClosureCandidateSelector.h>
#include <karto_sdk/Mapper.h>

#include "slam_toolbox/slam_mapper.hpp"

namespace slam_toolbox
{

class RemappingConfigurator
{
public:
  // Read `remapping_polygon` + `remapping_polygon_units` from `nh`.
  //   units="world"  → smapper is configured immediately and the loop-closure
  //                    filter is installed on `selector` (may be null).
  //   units="pixels" → polygon is queued until resolvePendingPolygon() runs
  //                    with the loaded grid dimensions.
  // Invalid / missing params are a no-op.  Errors are logged and ignored.
  void loadFromRosParams(ros::NodeHandle& nh,
                         mapper_utils::SMapper& smapper,
                         karto::LoopClosureCandidateSelector* selector);

  // Resolve a queued pixel-coord polygon to world coords using the grid
  // dimensions computed from `smapper`'s currently-loaded scans at
  // `resolution`, then configure smapper and install the filter on
  // `selector` (may be null).  No-op when no polygon is pending.
  void resolvePendingPolygon(mapper_utils::SMapper& smapper,
                             double resolution,
                             karto::LoopClosureCandidateSelector* selector);

private:
  std::optional<Polygon> pending_pixel_polygon_;
};

// Pin all nodes not belonging to the current remapping session during
// pose-graph optimisation.  Installed on the mapper, which forwards it
// to whichever scan solver is (or later becomes) attached.  The
// predicate reads through `smapper`; caller must keep smapper alive at
// least as long as the mapper.
void installFixedPosePredicate(mapper_utils::SMapper& smapper);

// Drop candidate scans whose recorded session no longer owns the cell
// at their position.  No-op when `selector` is null.  Safe to call
// repeatedly — replaces any previously-installed filter.
void installLoopClosureFilter(karto::LoopClosureCandidateSelector* selector,
                              mapper_utils::SMapper& smapper);

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_REMAPPING_CONFIGURATOR_H_
