/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/remapping/ownership_image.hpp"
#include "slam_toolbox/remapping/polygon_fill.hpp"
#include "slam_toolbox/remapping/remapping_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace slam_toolbox
{

namespace
{

struct BBox
{
  bool empty = true;
  kt_double min_x = std::numeric_limits<kt_double>::max();
  kt_double min_y = std::numeric_limits<kt_double>::max();
  kt_double max_x = std::numeric_limits<kt_double>::lowest();
  kt_double max_y = std::numeric_limits<kt_double>::lowest();

  void include(const std::vector<karto::Vector2<kt_double>>& polygon)
  {
    for (const auto& v : polygon)
    {
      min_x = std::min(min_x, v.GetX());
      min_y = std::min(min_y, v.GetY());
      max_x = std::max(max_x, v.GetX());
      max_y = std::max(max_y, v.GetY());
      empty = false;
    }
  }
};

}  // namespace

void OwnershipImage::build(
  const karto::Vector2<kt_double>& target_offset,
  kt_double resolution,
  const std::unordered_map<int, std::vector<std::vector<karto::Vector2<kt_double>>>>& session_polygons,
  int currentSessionId,
  const std::vector<std::vector<karto::Vector2<kt_double>>>& currentPolygons)
{
  // Compute the union bounding box over every polygon we'll paint.
  BBox bbox;
  for (const auto& [sid, polygons] : session_polygons)
  {
    if (sid != currentSessionId)
      for (const auto& polygon : polygons) bbox.include(polygon);
  }
  for (const auto& polygon : currentPolygons) bbox.include(polygon);

  if (bbox.empty)
  {
    image_.reset();
    return;
  }

  // Size the image to cover the bbox, using target_offset as origin so
  // target-grid cell indices are directly valid in the ownership image
  // (Option A alignment).  Width/height include the last painted cell:
  // fillSimplePolygon paints up to `floor(max_grid_coord)`,
  // so the required width is `floor(max_grid_coord) + 1`.  Cells outside
  // the resulting image are implicitly session 0 — see sessionAt().
  //
  // NOTE: cells at NEGATIVE grid coords (i.e. polygon vertices below
  // target_offset) are clipped — they fall outside this width/height
  // sizing and read as kBaseSessionId via sessionAt's bounds check.  By
  // design this matches buildRemapGrid's footprint-locking: the rendered
  // OccupancyGrid is itself sized to base_scans starting at target_offset
  // so it has no negative-index cells either, and any polygon claim below
  // target_offset would not be drawn into anyway.  If the ownership image
  // is ever consumed by a grid that does NOT share target_offset, this
  // assumption breaks and the build needs to be reworked to take a
  // separate own-origin offset.
  const kt_int32s width = std::max<kt_int32s>(
    0, static_cast<kt_int32s>(
         std::floor((bbox.max_x - target_offset.GetX()) / resolution)) + 1);
  const kt_int32s height = std::max<kt_int32s>(
    0, static_cast<kt_int32s>(
         std::floor((bbox.max_y - target_offset.GetY()) / resolution)) + 1);

  if (width <= 0 || height <= 0)
  {
    // Bbox lies entirely below the target origin — nothing to paint.
    image_.reset();
    return;
  }

  image_.reset(karto::Grid<kt_int32s>::CreateGrid(width, height, resolution));
  image_->GetCoordinateConverter()->SetOffset(target_offset);

  // Fill with the base session (owns everything initially).  Rows are
  // strided by WidthStep (width aligned up to 8), not width — see
  // Grid::GridIndex.  Using width here would leave the padding bytes at the
  // end of each row uninitialised and the filter would read garbage.
  kt_int32s* data = image_->GetDataPointer();
  const kt_int32s widthStep = image_->GetWidthStep();
  std::fill(data, data + (widthStep * height), kBaseSessionId);

  // Paint all polygons of a session by transforming each into grid coords
  // and delegating to the standalone scanline fill.  Same-session polygons
  // share a fill value, so overlaps between them are harmless.
  auto paintSession = [&](int session_id,
                          const std::vector<std::vector<karto::Vector2<kt_double>>>& polygons)
  {
    for (const auto& polygonWorld : polygons)
    {
      const auto polygonGrid = worldToGridPolygon(
        polygonWorld, target_offset, resolution);
      fillSimplePolygon<kt_int32s>(
        data, width, height, widthStep, polygonGrid, session_id);
    }
  };

  // Paint historical sessions ordered by session_id so later sessions
  // overwrite earlier ones in overlapping regions.
  std::map<int, const std::vector<std::vector<karto::Vector2<kt_double>>>*> historical;
  for (const auto& [sid, polygons] : session_polygons)
  {
    if (sid != currentSessionId)
    {
      historical[sid] = &polygons;
    }
  }

  for (const auto& [sid, polygonsPtr] : historical)
  {
    paintSession(sid, *polygonsPtr);
  }
  paintSession(currentSessionId, currentPolygons);
}

int OwnershipImage::sessionAt(const karto::Vector2<kt_int32s>& pt) const
{
  if (!image_) return kBaseSessionId;
  if (!image_->IsValidGridIndex(pt)) return kBaseSessionId;
  return image_->GetDataPointer()[image_->GridIndex(pt, false)];
}

int OwnershipImage::sessionAtWorld(const karto::Vector2<kt_double>& worldPos) const
{
  if (!image_) return kBaseSessionId;
  return sessionAt(image_->GetCoordinateConverter()->WorldToGrid(worldPos));
}

}  // namespace slam_toolbox
