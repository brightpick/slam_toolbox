/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/session_state.hpp"
#include "slam_toolbox/polygon_fill.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace slam_toolbox
{

// ---- Session labels ----

void SessionState::setSessionLabel(const SessionLabel& label)
{
  current_session_label_ = label;
}

void SessionState::registerNode(int unique_id)
{
  node_labels_[unique_id] = current_session_label_;
}

const SessionLabel* SessionState::getLabel(int unique_id) const
{
  auto it = node_labels_.find(unique_id);
  if (it == node_labels_.end()) return nullptr;
  return &it->second;
}

const std::unordered_map<int, SessionLabel>& SessionState::getAllLabels() const
{
  return node_labels_;
}

void SessionState::setAllLabels(const std::unordered_map<int, SessionLabel>& labels)
{
  node_labels_ = labels;

  // If remapping was configured before labels were loaded (typical startup
  // path: setParams → deserialize), recompute current_session_id now that the
  // real label history is visible.  Loaded labels may include session_ids
  // larger than whatever we computed against an empty map.
  if (remapping_)
  {
    remapping_->current_session_id = computeNextSessionId();
    current_session_label_.session_id = remapping_->current_session_id;
  }
}

// ---- Remapping config ----

int SessionState::computeNextSessionId() const
{
  int max_sid = 0;
  for (const auto& [node_id, label] : node_labels_)
  {
    max_sid = std::max(max_sid, label.session_id);
  }
  return max_sid + 1;
}

bool SessionState::setRemapping(std::vector<karto::Vector2<kt_double>> polygon)
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
  remapping_ = cfg;

  // Tag new scans with the computed session_id and carry the polygon on the
  // label so it is serialized to .labels on save.
  current_session_label_.session_id = cfg.current_session_id;
  current_session_label_.polygon = cfg.current_polygon;
  return true;
}

const std::optional<SessionState::RemappingConfig>& SessionState::getRemapping() const
{
  return remapping_;
}

bool SessionState::isRemappingNode(int unique_id) const
{
  if (!remapping_) return false;
  const SessionLabel* label = getLabel(unique_id);
  const int session_id = label ? label->session_id : 0;
  return session_id == remapping_->current_session_id;
}

// ---- Ownership image ----

void SessionState::buildOwnershipImage(kt_int32s width, kt_int32s height,
                                       const karto::Vector2<kt_double>& offset,
                                       kt_double resolution)
{
  if (!remapping_) return;
  ownership_image_.build(width, height, offset, resolution,
                         node_labels_,
                         remapping_->current_session_id,
                         remapping_->current_polygon);
}

}  // namespace slam_toolbox
