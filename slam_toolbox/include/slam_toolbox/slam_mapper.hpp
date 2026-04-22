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
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace mapper_utils
{

using namespace ::karto;

class SMapper
{
public:
  // Configuration for the active remapping session.
  // current_session_id is computed automatically by setRemapping() as
  // max(existing label.session_id) + 1 — it is never set by callers.
  // current_polygon is the area to re-map.  Must be a simple polygon
  // (edges don't cross themselves); non-convex is allowed.
  // Historical sessions are derived from the .labels file.
  struct RemappingConfig
  {
    int current_session_id;
    std::vector<karto::Vector2<kt_double>> current_polygon;
  };

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

  void setSessionLabel(const slam_toolbox::SessionLabel& label);
  void registerNode(int unique_id);
  const slam_toolbox::SessionLabel* getLabel(int unique_id) const;
  const std::unordered_map<int, slam_toolbox::SessionLabel>& getAllLabels() const;
  void setAllLabels(const std::unordered_map<int, slam_toolbox::SessionLabel>& labels);

  // Configure remapping with a simple polygon (edges must not cross
  // themselves; non-convex shapes are allowed).  Returns false and leaves
  // remapping unchanged if the polygon has < 3 vertices or self-intersects.
  // The session_id is computed as one more than the highest session_id
  // currently present in node_labels_, and is written to both the
  // RemappingConfig and the current session label so subsequent
  // registerNode() calls tag new scans with it.
  bool setRemapping(std::vector<karto::Vector2<kt_double>> polygon);
  const std::optional<RemappingConfig>& getRemapping() const;

  // Returns true if the node belongs to the current remapping session.
  bool isRemappingNode(int unique_id) const;

  // Returns the session_id that owns the cell at the given world position.
  // Returns 0 (base session) when no ownership image is built.
  int getOwnerAtWorldPosition(const karto::Vector2<kt_double>& position) const;

  // Build the ownership image from labels + current remapping config.
  // Must be called after labels are loaded (deserialization) and whenever
  // the grid dimensions change.
  void buildOwnershipImage(kt_int32s width, kt_int32s height,
                           const karto::Vector2<kt_double>& offset,
                           kt_double resolution);

  // Convenience: returns the ownership image pointer (may be null).
  const karto::Grid<kt_int32s>* getOwnershipImage() const { return ownership_image_.get(); }

protected:
  std::unique_ptr<karto::Mapper> mapper_;

private:
  int computeNextSessionId() const;

  std::unordered_map<int, slam_toolbox::SessionLabel> node_labels_;
  slam_toolbox::SessionLabel current_session_label_;
  std::optional<RemappingConfig> remapping_;
  std::unique_ptr<karto::Grid<kt_int32s>> ownership_image_;
};

} // end namespace

#endif //SLAM_TOOLBOX_SLAM_MAPPER_H_
