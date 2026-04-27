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
#include "slam_toolbox/remapping_state.hpp"

namespace mapper_utils
{

using namespace ::karto;

class SMapper
{
public:
  SMapper();
  ~SMapper();

  karto::OccupancyGrid* getOccupancyGrid(const double& resolution);

  tf2::Transform toTfPose(const karto::Pose2& pose) const;
  karto::Pose2 toKartoPose(const tf2::Transform& pose) const;

  void configure(const ros::NodeHandle& nh);
  void Reset();

  void setMapper(karto::Mapper* mapper);
  karto::Mapper* getMapper();

  void setCandidateSelector(karto::LoopClosureCandidateSelector* selector);

  void clearLocalizationBuffer();

  // Session labels, remapping config, and the ownership image — see
  // slam_toolbox/remapping_state.hpp.
  slam_toolbox::RemappingState& remappingState() { return remapping_state_; }
  const slam_toolbox::RemappingState& remappingState() const { return remapping_state_; }

protected:
  std::unique_ptr<karto::Mapper> mapper_;

private:
  // Builds an occupancy grid whose footprint is locked to the base-session
  // scans and renders every scan through the session-ownership filter.  Used
  // by getOccupancyGrid when a remapping polygon is active.
  karto::OccupancyGrid* buildRemapGrid(
    const karto::LocalizedRangeScanVector& scans, double resolution);

  slam_toolbox::RemappingState remapping_state_;
};

} // end namespace

#endif //SLAM_TOOLBOX_SLAM_MAPPER_H_
