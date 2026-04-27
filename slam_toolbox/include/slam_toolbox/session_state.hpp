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

#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include <karto_sdk/Karto.h>

#include "slam_toolbox/ownership_image.hpp"

namespace slam_toolbox
{

// Canonical shared types for session bookkeeping.  Defined here (the domain
// owner) and re-used by labels_serialization so there's a single source of
// truth for the map shapes that flow between save/load and SessionState.
using Polygon = std::vector<karto::Vector2<kt_double>>;
using NodeSessionMap = std::unordered_map<int, int>;
using SessionPolygonMap = std::unordered_map<int, Polygon>;

// Session id for the base map loaded from a .posegraph file.  Backwards
// compatibility: pre-remapping .posegraph files have no .labels sidecar, so
// every node implicitly belongs to session 0.  New remapping sessions are
// numbered starting at 1 and carry their own polygon.  Nodes missing from
// node_session_ids are treated as session 0.
constexpr int kBaseSessionId = 0;

class SessionState
{
public:
  // ---- Mutators ----

  // Tag the node with the currently-active session.  Tags as kBaseSessionId
  // before any setRemapping(), and as the active remap session afterwards.
  void registerNode(int node_id);

  // Configure remapping with a simple polygon (edges must not cross
  // themselves; non-convex shapes are allowed).  Returns false and leaves
  // remapping unchanged if the polygon has < 3 vertices or self-intersects.
  // On success an INFO log line is emitted with the new session id; the
  // session id itself is an internal detail and not exposed.  Subsequent
  // registerNode() calls tag new scans with it; the polygon is also
  // recorded in session_polygons so it is serialized to .labels on save.
  bool setRemapping(Polygon polygon);

  // Replace both maps in one shot (used after deserialization).  If a
  // remapping was configured earlier (via setRemapping) its session id is
  // re-picked as max(loaded session_id)+1 and its polygon re-registered
  // under the new id — so the caller never sees a session-id collision
  // with the just-loaded history.
  void setAll(const NodeSessionMap& node_session_ids,
              const SessionPolygonMap& session_polygons);

  // Build the ownership image from session_polygons + current remapping
  // polygon.  The image sizes itself to the polygon union bbox;
  // `target_offset` anchors its origin so target-grid cell indices remain
  // valid in it.  No-op when no remapping is active.
  void buildOwnershipImage(const karto::Vector2<kt_double>& target_offset,
                           kt_double resolution);

  // ---- Observers ----

  // Active remapping polygon, if one is configured.
  const std::optional<Polygon>& getRemappingPolygon() const { return remapping_polygon_; }

  // Per-node session ids and per-session polygons.  Read by the labels
  // serialization path.
  const NodeSessionMap& getAllNodeSessionIds() const { return node_session_ids_; }
  const SessionPolygonMap& getAllSessionPolygons() const { return session_polygons_; }

  // ---- Predicate factories ----

  // True when the node should be held fixed during pose-graph optimisation
  // (i.e. it does not belong to the active remapping session).  Always
  // false when no remapping is configured.  Captures *this by reference;
  // SessionState must outlive the returned callable.
  std::function<bool(int)> makeFixedPosePredicate() const;

  // True when a loop-closure candidate should be dropped (its recorded
  // session no longer owns the cell at its current corrected pose).
  // Captures *this by reference; SessionState must outlive the returned
  // callable.
  std::function<bool(karto::LocalizedRangeScan*)> makeLoopClosureFilter() const;

  // True when an occupancy-grid cell should be written by the given scan
  // during ray-tracing — i.e. when the scan and the cell agree on which
  // session owns the cell.  Caller must call buildOwnershipImage() with
  // the target grid's offset/resolution before using the returned
  // predicate.  Captures *this by reference; SessionState must outlive
  // the returned callable.
  std::function<bool(karto::LocalizedRangeScan*,
                     const karto::Vector2<kt_int32s>&)>
  makeGridCellPredicate() const;

private:
  int computeNextSessionId() const;
  int getSessionId(int node_id) const;

  NodeSessionMap node_session_ids_;
  SessionPolygonMap session_polygons_;
  int current_session_id_{kBaseSessionId};
  std::optional<Polygon> remapping_polygon_;
  OwnershipImage ownership_image_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_SESSION_STATE_H_
