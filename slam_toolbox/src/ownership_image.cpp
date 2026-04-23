/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/ownership_image.hpp"
#include "slam_toolbox/polygon_fill.hpp"

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
  const std::unordered_map<int, std::vector<karto::Vector2<kt_double>>>& session_polygons,
  int currentSessionId,
  const std::vector<karto::Vector2<kt_double>>& currentPolygon)
{
  // Compute the union bounding box over every polygon we'll paint.
  BBox bbox;
  for (const auto& [sid, polygon] : session_polygons)
  {
    if (sid != currentSessionId) bbox.include(polygon);
  }
  bbox.include(currentPolygon);

  if (bbox.empty)
  {
    image_.reset();
    return;
  }

  // Size the image to cover the bbox, using target_offset as origin so
  // target-grid cell indices are directly valid in the ownership image
  // (Option A alignment).  Width/height include the last painted cell:
  // polygon_fill::fillSimplePolygon paints up to `floor(max_grid_coord)`,
  // so the required width is `floor(max_grid_coord) + 1`.  Cells outside
  // the resulting image are implicitly session 0 — see sessionAt().
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

  // Fill with session 0 (base session owns everything initially).
  // Rows are strided by WidthStep (width aligned up to 8), not width — see
  // Grid::GridIndex.  Using width here would leave the padding bytes at the
  // end of each row uninitialised and the filter would read garbage.
  kt_int32s* data = image_->GetDataPointer();
  const kt_int32s widthStep = image_->GetWidthStep();
  std::fill(data, data + (widthStep * height), 0);

  // Paint a world-space polygon by transforming its vertices into grid
  // coords and delegating to the standalone scanline fill.
  auto paintPolygon = [&](int session_id,
                          const std::vector<karto::Vector2<kt_double>>& polyWorld)
  {
    const auto polyGrid = polygon_fill::worldToGridPolygon(
      polyWorld, target_offset, resolution);
    polygon_fill::fillSimplePolygon<kt_int32s>(
      data, width, height, widthStep, polyGrid, session_id);
  };

  // Paint historical sessions ordered by session_id so later sessions
  // overwrite earlier ones in overlapping regions.
  std::map<int, const std::vector<karto::Vector2<kt_double>>*> historical;
  for (const auto& [sid, polygon] : session_polygons)
  {
    if (sid != currentSessionId)
    {
      historical[sid] = &polygon;
    }
  }

  for (const auto& [sid, polyPtr] : historical)
  {
    paintPolygon(sid, *polyPtr);
  }
  paintPolygon(currentSessionId, currentPolygon);
}

int OwnershipImage::sessionAt(const karto::Vector2<kt_int32s>& pt) const
{
  if (!image_) return 0;
  if (!image_->IsValidGridIndex(pt)) return 0;
  return image_->GetDataPointer()[image_->GridIndex(pt, false)];
}

int OwnershipImage::sessionAtWorld(const karto::Vector2<kt_double>& worldPos) const
{
  if (!image_) return 0;
  return sessionAt(image_->GetCoordinateConverter()->WorldToGrid(worldPos));
}

}  // namespace slam_toolbox
