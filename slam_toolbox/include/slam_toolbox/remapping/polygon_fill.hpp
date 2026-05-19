/*
 * Copyright (c) 2026, Brightpick
 *
 * Standalone polygon utilities used by the remapping ownership image:
 *   - isSimplePolygon: validate a ring has ≥3 vertices and no crossing edges.
 *   - fillSimplePolygon: scanline even-odd fill into a strided byte/int grid.
 *
 * These are pulled out of slam_mapper.cpp so they can be tested directly
 * without spinning up a Mapper.
 */

#ifndef SLAM_TOOLBOX_POLYGON_FILL_H_
#define SLAM_TOOLBOX_POLYGON_FILL_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <karto_sdk/Karto.h>

namespace slam_toolbox
{

// Return false if the polygon is degenerate (< 3 vertices) or self-intersecting.
// Only strict crossings are detected — polygons whose edges merely touch (a
// shared vertex, a zero-length edge, or a collinear overlap) are accepted.
// Callers are expected to feed an already-deduplicated ring.
inline bool isSimplePolygon(
    const std::vector<karto::Vector2<kt_double>>& poly)
{
  const std::size_t n = poly.size();
  if (n < 3) return false;

  auto cross = [](const karto::Vector2<kt_double>& a,
                  const karto::Vector2<kt_double>& b,
                  const karto::Vector2<kt_double>& c) -> double
  {
    return (b.GetX() - a.GetX()) * (c.GetY() - a.GetY())
         - (b.GetY() - a.GetY()) * (c.GetX() - a.GetX());
  };

  for (std::size_t i = 0; i < n; ++i)
  {
    for (std::size_t j = i + 2; j < n; ++j)
    {
      // Skip wrap-around adjacency: edges 0 and n-1 share poly[0].
      if (i == 0 && j == n - 1) continue;

      const auto& a = poly[i];
      const auto& b = poly[(i + 1) % n];
      const auto& c = poly[j];
      const auto& d = poly[(j + 1) % n];

      const double d1 = cross(a, b, c);
      const double d2 = cross(a, b, d);
      const double d3 = cross(c, d, a);
      const double d4 = cross(c, d, b);

      if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
          ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
      {
        return false;
      }
    }
  }
  return true;
}

// Scanline even-odd fill for a simple polygon (convex or non-convex).
//
// Polygon is in grid-cell coordinates (may be fractional).  For each integer
// scanline y, intersects all non-horizontal edges with y, sorts the
// x-intersections, and fills between consecutive pairs (1st-2nd, 3rd-4th,
// ...).  The half-open y-rule (edge counts scanline y iff one endpoint is
// at-or-below y and the other strictly above) avoids double-counting when
// two edges meet at a vertex with integer y.
//
// Writes `fillValue` into data[y * widthStep + x] for every cell inside the
// polygon.  Cells outside [0, width) × [0, height) are clipped away.
// Rows are assumed strided by `widthStep` which may exceed `width` (byte
// alignment); see karto::Grid::GridIndex.
//
// Polygons with fewer than 3 vertices are silently ignored — validate with
// isSimplePolygon() before calling if you care.
template <typename CellT>
void fillSimplePolygon(
    CellT* data,
    kt_int32s width,
    kt_int32s height,
    kt_int32s widthStep,
    const std::vector<karto::Vector2<kt_double>>& polygon,
    CellT fillValue)
{
  if (polygon.size() < 3 || width <= 0 || height <= 0) return;

  double yMin = std::numeric_limits<double>::infinity();
  double yMax = -std::numeric_limits<double>::infinity();
  for (const auto& v : polygon)
  {
    yMin = std::min(yMin, v.GetY());
    yMax = std::max(yMax, v.GetY());
  }

  const kt_int32s yStart =
    std::max(static_cast<kt_int32s>(0), static_cast<kt_int32s>(std::ceil(yMin)));
  const kt_int32s yEnd =
    std::min(static_cast<kt_int32s>(height - 1),
             static_cast<kt_int32s>(std::floor(yMax)));

  std::vector<double> xs;
  const std::size_t n = polygon.size();
  for (kt_int32s y = yStart; y <= yEnd; ++y)
  {
    xs.clear();
    const double yd = static_cast<double>(y);
    for (std::size_t i = 0; i < n; ++i)
    {
      const auto& p = polygon[i];
      const auto& q = polygon[(i + 1) % n];
      const double py = p.GetY();
      const double qy = q.GetY();
      if ((py <= yd && qy > yd) || (qy <= yd && py > yd))
      {
        const double t = (yd - py) / (qy - py);
        xs.push_back(p.GetX() + t * (q.GetX() - p.GetX()));
      }
    }
    std::sort(xs.begin(), xs.end());
    for (std::size_t i = 0; i + 1 < xs.size(); i += 2)
    {
      const kt_int32s xStart = std::max(
        static_cast<kt_int32s>(0),
        static_cast<kt_int32s>(std::ceil(xs[i])));
      const kt_int32s xEnd = std::min(
        static_cast<kt_int32s>(width - 1),
        static_cast<kt_int32s>(std::floor(xs[i + 1])));
      for (kt_int32s x = xStart; x <= xEnd; ++x)
      {
        data[y * widthStep + x] = fillValue;
      }
    }
  }
}

// Convenience: transform a world-space polygon to grid coordinates using the
// standard world = offset + grid * resolution model.
inline std::vector<karto::Vector2<kt_double>> worldToGridPolygon(
    const std::vector<karto::Vector2<kt_double>>& polygonWorld,
    const karto::Vector2<kt_double>& offset,
    kt_double resolution)
{
  std::vector<karto::Vector2<kt_double>> out;
  out.reserve(polygonWorld.size());
  for (const auto& v : polygonWorld)
  {
    out.emplace_back((v.GetX() - offset.GetX()) / resolution,
                     (v.GetY() - offset.GetY()) / resolution);
  }
  return out;
}

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_POLYGON_FILL_H_
