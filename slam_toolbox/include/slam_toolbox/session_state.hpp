/*
 * Copyright (c) 2026, Brightpick
 *
 * Holds the "which scans belong to which session" state that underpins
 * the remapping feature:
 *   - a per-pose label map (which session a scan was acquired in),
 *   - the currently-active remapping config (area + assigned session id),
 *   - the ownership image derived from labels + polygons.
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
#include "slam_toolbox/session_label.hpp"

namespace slam_toolbox
{

class SessionState
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

  // ---- Session labels ----

  void setSessionLabel(const SessionLabel& label);
  void registerNode(int unique_id);
  const SessionLabel* getLabel(int unique_id) const;
  const std::unordered_map<int, SessionLabel>& getAllLabels() const;
  void setAllLabels(const std::unordered_map<int, SessionLabel>& labels);

  // ---- Remapping config ----

  // Configure remapping with a simple polygon (edges must not cross
  // themselves; non-convex shapes are allowed).  Returns false and leaves
  // remapping unchanged if the polygon has < 3 vertices or self-intersects.
  // The session_id is computed as one more than the highest session_id
  // currently present in the label map, and is written to both the
  // RemappingConfig and the current session label so subsequent
  // registerNode() calls tag new scans with it.
  bool setRemapping(std::vector<karto::Vector2<kt_double>> polygon);
  const std::optional<RemappingConfig>& getRemapping() const;

  // Returns true if the node belongs to the current remapping session.
  bool isRemappingNode(int unique_id) const;

  // ---- Ownership image ----

  // Build the ownership image from labels + current remapping config.
  // Must be called after labels are loaded (deserialization) and whenever
  // the grid dimensions change.  No-op when no remapping is active.
  void buildOwnershipImage(kt_int32s width, kt_int32s height,
                           const karto::Vector2<kt_double>& offset,
                           kt_double resolution);

  // Access the ownership image (read-only).  Use `.ownerAtWorld(pos)` to
  // query cell ownership and `.grid()` to get the raw karto grid.
  const OwnershipImage& ownershipImage() const { return ownership_image_; }

private:
  int computeNextSessionId() const;

  std::unordered_map<int, SessionLabel> node_labels_;
  SessionLabel current_session_label_;
  std::optional<RemappingConfig> remapping_;
  OwnershipImage ownership_image_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_SESSION_STATE_H_
