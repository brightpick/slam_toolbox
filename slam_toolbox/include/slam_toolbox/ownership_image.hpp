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
 * by session 0 — that contract lives in karto::IsOwnedBy.
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
  // the resulting image are treated as session 0 by karto::IsOwnedBy.
  void build(const karto::Vector2<kt_double>& target_offset,
             kt_double resolution,
             const std::unordered_map<int, std::vector<karto::Vector2<kt_double>>>& session_polygons,
             int currentSessionId,
             const std::vector<karto::Vector2<kt_double>>& currentPolygon);

  // Session owning the cell at `worldPos`.  Returns 0 when no image is
  // built or the position is out of bounds.
  int ownerAtWorld(const karto::Vector2<kt_double>& worldPos) const;

  // Raw grid pointer (may be null).  Exposed for karto ray tracing, which
  // needs the Grid API directly.
  const karto::Grid<kt_int32s>* grid() const { return image_.get(); }

  // Drop the image.
  void reset() { image_.reset(); }

private:
  std::unique_ptr<karto::Grid<kt_int32s>> image_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_OWNERSHIP_IMAGE_H_
