/*
 * Copyright (c) 2026, Brightpick
 *
 * Per-cell session ownership grid used by the remapping filter.  Each cell
 * stores the session_id that "owns" it; session 0 is the base map and owns
 * any cell not covered by a polygon.  The image is rebuilt whenever the
 * underlying occupancy grid's dimensions change.
 *
 * Lifted out of slam_mapper so it can be unit-tested without a Mapper and
 * so the remapping code sits behind a narrow interface.
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
  // Build a fresh image sized to (width x height) with the given world
  // offset / resolution.  Historical sessions (entries in session_polygons
  // whose session_id != currentSessionId) are painted first in session-id
  // order, then currentPolygon paints last so it wins every overlap.
  // Cells outside all polygons remain session 0.
  void build(kt_int32s width, kt_int32s height,
             const karto::Vector2<kt_double>& offset,
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
