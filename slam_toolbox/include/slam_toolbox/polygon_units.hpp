/*
 * Copyright (c) 2026, Brightpick
 *
 * Pure conversion between PGM-pixel and world coordinates for remapping
 * polygons.  Lives outside RemappingState to keep that class free of any
 * Karto Mapper / OccupancyGrid coupling — the caller computes grid
 * dimensions and passes them in here.
 *
 * PGM convention: py=0 is the TOP row, y increasing downward.  Karto's
 * grid row 0 is the bottom row, so an image_row → grid_row mapping is
 * `(height - 1 - py)`.
 *
 *   world_x = offset.x + px * resolution
 *   world_y = offset.y + (height - 1 - py) * resolution
 */

#ifndef SLAM_TOOLBOX_POLYGON_UNITS_H_
#define SLAM_TOOLBOX_POLYGON_UNITS_H_

#include <karto_sdk/Karto.h>

#include "slam_toolbox/remapping_state.hpp"

namespace slam_toolbox
{
namespace polygon_units
{

// Convert a polygon expressed in PGM-pixel coords to world coords using
// the target grid's `offset`, `resolution`, and pixel `height`.
Polygon pixelToWorld(const Polygon& pixel_poly,
                     const karto::Vector2<kt_double>& offset,
                     kt_double resolution,
                     kt_int32s height);

}  // namespace polygon_units
}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_POLYGON_UNITS_H_
