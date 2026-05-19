/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/remapping/polygon_units.hpp"

namespace slam_toolbox
{

Polygon pixelPolygonToWorld(const Polygon& pixel_poly,
                            const karto::Vector2<kt_double>& offset,
                            kt_double resolution,
                            kt_int32s height)
{
  Polygon world;
  world.reserve(pixel_poly.size());
  for (const auto& v : pixel_poly)
  {
    const kt_double wx = offset.GetX() + v.GetX() * resolution;
    const kt_double wy = offset.GetY() + (height - 1 - v.GetY()) * resolution;
    world.emplace_back(wx, wy);
  }
  return world;
}

}  // namespace slam_toolbox
