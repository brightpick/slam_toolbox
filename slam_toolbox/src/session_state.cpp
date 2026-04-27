/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/session_state.hpp"
#include "slam_toolbox/polygon_fill.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace slam_toolbox
{

SessionState::SessionState(NodeSessionMap node_session_ids,
                           SessionPolygonMap session_polygons)
  : node_session_ids_(std::move(node_session_ids)),
    session_polygons_(std::move(session_polygons))
{
}

// ---- Mutators ----

void SessionState::registerNode(int node_id)
{
  node_session_ids_[node_id] = current_session_id_;
}

bool SessionState::setRemapping(Polygon polygon)
{
  if (!polygon_fill::isSimplePolygon(polygon))
  {
    ROS_ERROR("SessionState::setRemapping: rejected polygon with %zu vertices "
              "— it must have at least 3 vertices and must not self-intersect.",
              polygon.size());
    return false;
  }

  // Tag new scans with the computed session_id and record the polygon so
  // it is serialized to .labels on save.
  current_session_id_ = computeNextSessionId();
  session_polygons_[current_session_id_] = polygon;
  ROS_INFO("SessionState: remapping session %d configured "
           "(%zu-vertex polygon).", current_session_id_, polygon.size());
  remapping_polygon_ = std::move(polygon);
  return true;
}

void SessionState::buildOwnershipImage(const karto::Vector2<kt_double>& target_offset,
                                       kt_double resolution)
{
  if (!remapping_polygon_) return;
  ownership_image_.build(target_offset, resolution,
                         session_polygons_,
                         current_session_id_,
                         *remapping_polygon_);
}

// ---- Predicate factories ----

std::function<bool(int)> SessionState::makeFixedPosePredicate() const
{
  return [this](int id) {
    return remapping_polygon_.has_value() &&
           getSessionId(id) != current_session_id_;
  };
}

std::function<bool(karto::LocalizedRangeScan*)>
SessionState::makeLoopClosureFilter() const
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
SessionState::makeGridCellPredicate() const
{
  return [this](karto::LocalizedRangeScan* pScan,
                const karto::Vector2<kt_int32s>& cell) -> bool {
    return ownership_image_.sessionAt(cell) ==
           getSessionId(pScan->GetUniqueId());
  };
}

// ---- Internals ----

int SessionState::computeNextSessionId() const
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

int SessionState::getSessionId(int node_id) const
{
  auto it = node_session_ids_.find(node_id);
  return it != node_session_ids_.end() ? it->second : kBaseSessionId;
}

}  // namespace slam_toolbox
