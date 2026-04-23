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

void SessionState::registerNode(int node_id)
{
  node_session_ids_[node_id] = current_session_id_;
}

void SessionState::tagNode(int node_id, int session_id)
{
  node_session_ids_[node_id] = session_id;
}

int SessionState::getSessionId(int node_id) const
{
  auto it = node_session_ids_.find(node_id);
  return it != node_session_ids_.end() ? it->second : kBaseSessionId;
}

void SessionState::setAll(const NodeSessionMap& node_session_ids,
                          const SessionPolygonMap& session_polygons)
{
  node_session_ids_ = node_session_ids;
  session_polygons_ = session_polygons;

  // If remapping was configured before loading, re-pick its session id to
  // avoid collision with the just-loaded history and re-register its
  // polygon under the new id.  No-op when no remapping is active.
  if (!remapping_polygon_) return;
  current_session_id_ = computeNextSessionId();
  session_polygons_[current_session_id_] = *remapping_polygon_;
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

  // Tag new scans with the computed session_id and record the polygon so
  // it is serialized to .labels on save.
  current_session_id_ = computeNextSessionId();
  session_polygons_[current_session_id_] = polygon;
  remapping_polygon_ = std::move(polygon);
  return true;
}

bool SessionState::isRemappingNode(int node_id) const
{
  if (!remapping_polygon_) return false;
  return getSessionId(node_id) == current_session_id_;
}

// ---- Ownership image ----

void SessionState::buildOwnershipImage(const karto::Vector2<kt_double>& target_offset,
                                       kt_double resolution)
{
  if (!remapping_polygon_) return;
  ownership_image_.build(target_offset, resolution,
                         session_polygons_,
                         current_session_id_,
                         *remapping_polygon_);
}

int SessionState::ownerAt(const karto::Vector2<kt_int32s>& cell) const
{
  return ownership_image_.sessionAt(cell);
}

int SessionState::ownerAtWorld(const karto::Vector2<kt_double>& world_pos) const
{
  return ownership_image_.sessionAtWorld(world_pos);
}

}  // namespace slam_toolbox
