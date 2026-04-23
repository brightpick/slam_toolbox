/*
 * Copyright (c) 2026, Brightpick
 *
 * Per-cell session ownership grid used by the remapping filter.  Each cell
 * stores the session_id that "owns" it.  The grid sizes itself to the union
 * bounding box of the input polygons (plus one row/column of margin) so the
 * memory cost is proportional to the remap area, not to the full map.
 *
 * Alignment: the grid shares `offset` and `resolution` with the target
 * occupancy grid so target-grid cell indices can index into the ownership
 * image directly.  Cells OUTSIDE the ownership image are implicitly owned
 * by session 0 — `sessionAt()` encodes that contract.
 */

#ifndef SLAM_TOOLBOX_OWNERSHIP_IMAGE_H_
#define SLAM_TOOLBOX_OWNERSHIP_IMAGE_H_

#include <karto_sdk/Karto.h>
#include <memory>
#include <unordered_map>
#include <vector>

namespace slam_toolbox
{

class OwnershipImage
{
public:
  // Build a fresh image sized to the union bbox of all polygons
  // (historical + current), aligned to `target_offset` at `resolution`.
  // Historical sessions (entries in session_polygons whose session_id !=
  // currentSessionId) are painted first in session-id order, then
  // currentPolygon paints last so it wins every overlap.  Cells outside
  // the resulting image are treated as session 0 by `sessionAt()`.
  void build(const karto::Vector2<kt_double>& target_offset,
             kt_double resolution,
             const std::unordered_map<int, std::vector<karto::Vector2<kt_double>>>& session_polygons,
             int currentSessionId,
             const std::vector<karto::Vector2<kt_double>>& currentPolygon);

  // Session owning the cell at grid index `pt` (indices are in target-grid
  // coords, which this image shares by virtue of using target_offset).
  // Returns 0 when no image is built or the cell is out of bounds —
  // unpainted periphery is implicitly session 0.
  int sessionAt(const karto::Vector2<kt_int32s>& pt) const;

  // Session owning the cell at world position `worldPos`.  Converts to
  // grid coords then delegates to sessionAt.  Same return contract.
  int sessionAtWorld(const karto::Vector2<kt_double>& worldPos) const;

  // Drop the image.
  void reset() { image_.reset(); }

private:
  std::unique_ptr<karto::Grid<kt_int32s>> image_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_OWNERSHIP_IMAGE_H_
