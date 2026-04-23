/*
 * Copyright (c) 2026, Brightpick
 *
 * Holds the "which scans belong to which session" state that underpins
 * the remapping feature:
 *   - node_session_ids: per-pose session id (pose_id → session_id),
 *   - session_polygons: area of each session (session_id → polygon),
 *   - the currently-active remapping config (area + assigned session id),
 *   - the ownership image derived from the above.
 *
 * Lifted out of SMapper so this state and its API live in one place and
 * the original SMapper has minimal feature-specific churn.
 */

#ifndef SLAM_TOOLBOX_SESSION_STATE_H_
#define SLAM_TOOLBOX_SESSION_STATE_H_

#include <optional>
#include <unordered_map>
#include <vector>

#include <karto_sdk/Karto.h>

#include "slam_toolbox/ownership_image.hpp"

namespace slam_toolbox
{

class SessionState
{
public:
  using Polygon = std::vector<karto::Vector2<kt_double>>;
  using NodeSessionMap = std::unordered_map<int, int>;
  using SessionPolygonMap = std::unordered_map<int, Polygon>;

  // Configuration for the active remapping session.
  // current_session_id is computed automatically by setRemapping() as
  // max(existing session_id) + 1 — it is never set by callers.
  // current_polygon is the area to re-map.  Must be a simple polygon
  // (edges don't cross themselves); non-convex is allowed.
  // Historical sessions are derived from the .labels file.
  struct RemappingConfig
  {
    int current_session_id;
    Polygon current_polygon;
  };

  // ---- Session ids ----

  // Session id assigned to new scans registered via registerNode().
  void setCurrentSessionId(int session_id);
  void registerNode(int node_id);

  // Session id for `node_id`, or 0 (base session) if unknown.
  int getSessionId(int node_id) const;

  const NodeSessionMap& getAllNodeSessionIds() const { return node_session_ids_; }
  const SessionPolygonMap& getAllSessionPolygons() const { return session_polygons_; }

  // Replace both maps in one shot (used after deserialization).
  void setAll(const NodeSessionMap& node_session_ids,
              const SessionPolygonMap& session_polygons);

  // ---- Remapping config ----

  // Configure remapping with a simple polygon (edges must not cross
  // themselves; non-convex shapes are allowed).  Returns false and leaves
  // remapping unchanged if the polygon has < 3 vertices or self-intersects.
  // The session_id is computed as one more than the highest session_id
  // currently present, and is written to both the RemappingConfig and the
  // current session id so subsequent registerNode() calls tag new scans
  // with it.  The polygon is also recorded in session_polygons so it is
  // serialized to .labels on save.
  bool setRemapping(Polygon polygon);
  const std::optional<RemappingConfig>& getRemapping() const { return remapping_; }

  // Returns true if the node belongs to the current remapping session.
  bool isRemappingNode(int node_id) const;

  // ---- Ownership image ----

  // Build the ownership image from session_polygons + current remapping
  // config.  The image sizes itself to the polygon union bbox; `target_offset`
  // anchors its origin so target-grid cell indices remain valid in it.
  // No-op when no remapping is active.
  void buildOwnershipImage(const karto::Vector2<kt_double>& target_offset,
                           kt_double resolution);

  // Access the ownership image (read-only).
  const OwnershipImage& ownershipImage() const { return ownership_image_; }

private:
  int computeNextSessionId() const;

  NodeSessionMap node_session_ids_;
  SessionPolygonMap session_polygons_;
  int current_session_id_{0};
  std::optional<RemappingConfig> remapping_;
  OwnershipImage ownership_image_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_SESSION_STATE_H_
