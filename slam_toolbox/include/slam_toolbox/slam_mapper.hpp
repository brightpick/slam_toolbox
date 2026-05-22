/*
 * Author
 * Copyright (c) 2019 Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX_SLAM_MAPPER_H_
#define SLAM_TOOLBOX_SLAM_MAPPER_H_

#include "ros/ros.h"
#include "karto_sdk/Mapper.h"
#include "karto_sdk/Karto.h"
#include "tf2/utils.h"
#include "slam_toolbox/remapping/remapping_state.hpp"

namespace mapper_utils
{

using namespace ::karto;

class SMapper
{
public:
  SMapper();
  ~SMapper();

  // get occupancy grid from scans
  karto::OccupancyGrid* getOccupancyGrid(const double& resolution);

  // convert Karto pose to TF pose
  tf2::Transform toTfPose(const karto::Pose2& pose) const;

  // convert TF pose to karto pose
  karto::Pose2 toKartoPose(const tf2::Transform& pose) const;

  void configure(const ros::NodeHandle& nh);
  void Reset();

  // // processors
  // kt_bool ProcessAtDock(LocalizedRangeScan* pScan);
  // kt_bool ProcessAgainstNode(LocalizedRangeScan* pScan,  const int& nodeId);
  // kt_bool ProcessAgainstNodesNearBy(LocalizedRangeScan* pScan);
  // kt_bool ProcessLocalization(LocalizedRangeScan* pScan);

  void setMapper(karto::Mapper* mapper);
  karto::Mapper* getMapper();

  void clearLocalizationBuffer();

  // Session labels, remapping config, and the ownership image — see
  // slam_toolbox/remapping/remapping_state.hpp.
  slam_toolbox::RemappingState& remappingState() { return remapping_state_; }
  const slam_toolbox::RemappingState& remappingState() const { return remapping_state_; }

  // Rebuild the ownership image from current scans + the currently-active
  // remapping polygon, using the base-session footprint as the anchor so its
  // origin matches the grid produced by getOccupancyGrid().  Call this right
  // after activating a remapping session — otherwise the loop-closure filter
  // sees a null image (sessionAtWorld returns kBaseSessionId for every query)
  // until the next map publish runs buildRemapGrid, which silently drops
  // every current-session scan as a candidate.  No-op when no remapping is
  // active or when no base-session scans exist.
  void rebuildOwnershipImage(double resolution);

  // Width/height/offset of the occupancy grid that buildRemapGrid would
  // render at this resolution — i.e. the base-session footprint.  Use this
  // (not ComputeDimensions over all scans) to convert pixel coords from a
  // published map into world coords: the published PGM is rendered against
  // this frame, and on subsequent remap cycles the all-scans bbox can drift
  // (newer-session scans may extend the bbox), which would shift the
  // origin.  Returns false when no base-session scans are present.
  bool getBaseFootprint(double resolution,
                        kt_int32s& width, kt_int32s& height,
                        karto::Vector2<kt_double>& offset) const;

protected:
  std::unique_ptr<karto::Mapper> mapper_;

private:
  // Builds an occupancy grid whose footprint is locked to the base-session
  // scans and renders every scan through the session-ownership filter.  Used
  // by getOccupancyGrid when a remapping polygon is active.
  karto::OccupancyGrid* buildRemapGrid(
    const karto::LocalizedRangeScanVector& scans, double resolution);

  // Filter scans to the base session and compute their occupancy-grid
  // footprint (ComputeDimensions).  Returns false when no base scans are
  // present, in which case width/height/offset are not set.
  bool computeBaseFootprint(
    const karto::LocalizedRangeScanVector& scans, double resolution,
    kt_int32s& width, kt_int32s& height,
    karto::Vector2<kt_double>& offset) const;

  slam_toolbox::RemappingState remapping_state_;
};

} // end namespace

#endif //SLAM_TOOLBOX_SLAM_MAPPER_H_
