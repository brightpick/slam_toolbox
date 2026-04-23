/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/session_state.hpp"
#include "slam_toolbox/polygon_fill.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace slam_toolbox
{

// ---- Session ids ----

void SessionState::setCurrentSessionId(int session_id)
{
  current_session_id_ = session_id;
}

void SessionState::registerNode(int node_id)
{
  node_session_ids_[node_id] = current_session_id_;
}

int SessionState::getSessionId(int node_id) const
{
  auto it = node_session_ids_.find(node_id);
  return it != node_session_ids_.end() ? it->second : 0;
}

void SessionState::setAll(const NodeSessionMap& node_session_ids,
                          const SessionPolygonMap& session_polygons)
{
  node_session_ids_ = node_session_ids;
  session_polygons_ = session_polygons;

  // If remapping was configured before labels were loaded (typical startup
  // path: setParams → deserialize), recompute current_session_id now that the
  // real session history is visible.  Loaded data may include session_ids
  // larger than whatever we computed against an empty map.
  if (remapping_)
  {
    remapping_->current_session_id = computeNextSessionId();
    current_session_id_ = remapping_->current_session_id;
    session_polygons_[remapping_->current_session_id] = remapping_->current_polygon;
  }
}

// ---- Remapping config ----

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

bool SessionState::setRemapping(Polygon polygon)
{
  if (!polygon_fill::isSimplePolygon(polygon))
  {
    ROS_ERROR("SessionState::setRemapping: rejected polygon with %zu vertices "
              "— it must have at least 3 vertices and must not self-intersect.",
              polygon.size());
    return false;
  }

  RemappingConfig cfg;
  cfg.current_session_id = computeNextSessionId();
  cfg.current_polygon = std::move(polygon);

  // Tag new scans with the computed session_id and record the polygon so
  // it is serialized to .labels on save.
  current_session_id_ = cfg.current_session_id;
  session_polygons_[cfg.current_session_id] = cfg.current_polygon;
  remapping_ = std::move(cfg);
  return true;
}

bool SessionState::isRemappingNode(int node_id) const
{
  if (!remapping_) return false;
  return getSessionId(node_id) == remapping_->current_session_id;
}

// ---- Ownership image ----

void SessionState::buildOwnershipImage(kt_int32s width, kt_int32s height,
                                       const karto::Vector2<kt_double>& offset,
                                       kt_double resolution)
{
  if (!remapping_) return;
  ownership_image_.build(width, height, offset, resolution,
                         session_polygons_,
                         remapping_->current_session_id,
                         remapping_->current_polygon);
}

}  // namespace slam_toolbox
