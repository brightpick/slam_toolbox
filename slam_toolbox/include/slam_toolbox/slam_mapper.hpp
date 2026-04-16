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
#include "slam_toolbox/session_label.hpp"
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace mapper_utils
{

using namespace ::karto;

class SMapper
{
public:
  // Configuration for the spatial remapping filter.
  // Both fields must be provided together — remapping requires a spatial
  // boundary AND a set of session IDs that define the remapping session.
  // non_fixed_session_ids also controls which poses Ceres is allowed to move.
  struct RemappingConfig
  {
    karto::BoundingBox2 bbox;
    std::unordered_set<int> non_fixed_session_ids;
  };

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

  void setCandidateSelector(karto::LoopClosureCandidateSelector* selector);

  void clearLocalizationBuffer();

  void setSessionLabel(const slam_toolbox::SessionLabel& label);
  void registerNode(int unique_id);
  const slam_toolbox::SessionLabel* getLabel(int unique_id) const;
  const std::unordered_map<int, slam_toolbox::SessionLabel>& getAllLabels() const;
  void setAllLabels(const std::unordered_map<int, slam_toolbox::SessionLabel>& labels);

  // Configure spatial remapping. bbox defines the remapped area; scans whose
  // session_id is in non_fixed_session_ids may only draw inside it, all others
  // only outside. The same session IDs also control which Ceres poses are free
  // to move during optimisation.
  void setRemapping(RemappingConfig config);

  const std::optional<RemappingConfig>& getRemapping() const;

protected:
  std::unique_ptr<karto::Mapper> mapper_;

private:
  std::unordered_map<int, slam_toolbox::SessionLabel> node_labels_;
  slam_toolbox::SessionLabel current_session_label_;
  std::optional<RemappingConfig> remapping_;
};

} // end namespace

#endif //SLAM_TOOLBOX_SLAM_MAPPER_H_
