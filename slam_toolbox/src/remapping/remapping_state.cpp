/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/remapping/remapping_state.hpp"
#include "slam_toolbox/remapping/polygon_fill.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace slam_toolbox
{

RemappingState::RemappingState(NodeSessionMap node_session_ids,
                               SessionPolygonMap session_polygons)
  : node_session_ids_(std::move(node_session_ids)),
    session_polygons_(std::move(session_polygons))
{
}

// ---- Mutators ----

void RemappingState::registerNode(int node_id)
{
  node_session_ids_[node_id] = current_session_id_;
}

bool RemappingState::setRemapping(MultiPolygon polygons)
{
  if (polygons.empty())
  {
    ROS_ERROR("RemappingState::setRemapping: rejected empty polygon set "
              "— at least one polygon is required.");
    return false;
  }
  for (const auto& polygon : polygons)
  {
    if (!isSimplePolygon(polygon))
    {
      ROS_ERROR("RemappingState::setRemapping: rejected polygon with %zu "
                "vertices — each polygon must have at least 3 vertices and "
                "must not self-intersect.", polygon.size());
      return false;
    }
  }

  // Tag new scans with the computed session_id and record the polygons so
  // they are serialized to .labels on save.
  current_session_id_ = computeNextSessionId();
  session_polygons_[current_session_id_] = polygons;
  ROS_INFO("RemappingState: remapping session %d configured "
           "(%zu polygon(s)).", current_session_id_, polygons.size());
  remapping_polygons_ = std::move(polygons);
  return true;
}

void RemappingState::buildOwnershipImage(const karto::Vector2<kt_double>& target_offset,
                                         kt_double resolution)
{
  if (!remapping_polygons_) return;
  ownership_image_.build(target_offset, resolution,
                         session_polygons_,
                         current_session_id_,
                         *remapping_polygons_);
}

// ---- Predicate factories ----

std::function<bool(int)> RemappingState::makeFixedPosePredicate() const
{
  return [this](int id) {
    if (remapping_polygons_.has_value())
    {
      return getSessionId(id) != current_session_id_;
    }
    return !session_polygons_.empty();
  };
}

std::function<bool(int)> RemappingState::makeComputeGridSizePredicate() const
{
  return [this](int id) {
    return getSessionId(id) == kBaseSessionId;
  };
}

std::function<bool(karto::LocalizedRangeScan*)>
RemappingState::makeLoopClosureFilter() const
{
  return [this](karto::LocalizedRangeScan* pScan) -> bool {
    const int scan_sid = getSessionId(pScan->GetUniqueId());
    const int owner = ownership_image_.sessionAtWorld(
      pScan->GetCorrectedPose().GetPosition());
    return scan_sid != owner;
  };
}

std::function<bool(karto::LocalizedRangeScan*,
                   const karto::Vector2<kt_int32s>&)>
RemappingState::makeGridCellPredicate() const
{
  return [this](karto::LocalizedRangeScan* pScan,
                const karto::Vector2<kt_int32s>& cell) -> bool {
    return ownership_image_.sessionAt(cell) ==
           getSessionId(pScan->GetUniqueId());
  };
}

// ---- Internals ----

int RemappingState::computeNextSessionId() const
{
  int max_sid = 0;
  for (const auto& [node_id, session_id] : node_session_ids_)
  {
    max_sid = std::max(max_sid, session_id);
  }
  for (const auto& [session_id, polygon] : session_polygons_)
  {
    max_sid = std::max(max_sid, session_id);
  }
  return max_sid + 1;
}

int RemappingState::getSessionId(int node_id) const
{
  auto it = node_session_ids_.find(node_id);
  return it != node_session_ids_.end() ? it->second : kBaseSessionId;
}

}  // namespace slam_toolbox
