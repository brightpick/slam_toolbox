/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/ownership_image.hpp"
#include "slam_toolbox/polygon_fill.hpp"

#include <algorithm>
#include <map>

namespace slam_toolbox
{

void OwnershipImage::build(
  kt_int32s width, kt_int32s height,
  const karto::Vector2<kt_double>& offset,
  kt_double resolution,
  const std::unordered_map<int, std::vector<karto::Vector2<kt_double>>>& session_polygons,
  int currentSessionId,
  const std::vector<karto::Vector2<kt_double>>& currentPolygon)
{
  image_.reset(karto::Grid<kt_int32s>::CreateGrid(width, height, resolution));
  image_->GetCoordinateConverter()->SetOffset(offset);

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
      polyWorld, offset, resolution);
    polygon_fill::fillSimplePolygon<kt_int32s>(
      data, width, height, widthStep, polyGrid, session_id);
  };

  // Paint historical sessions (everything except the current one) ordered
  // by session_id so later sessions overwrite earlier ones in overlapping
  // regions.  std::map gives us that ordering from the unordered input.
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

int OwnershipImage::ownerAtWorld(const karto::Vector2<kt_double>& worldPos) const
{
  if (!image_) return 0;
  const karto::Vector2<kt_int32s> gridIdx =
    image_->GetCoordinateConverter()->WorldToGrid(worldPos);
  if (!image_->IsValidGridIndex(gridIdx)) return 0;
  return image_->GetDataPointer()[image_->GridIndex(gridIdx, false)];
}

}  // namespace slam_toolbox
