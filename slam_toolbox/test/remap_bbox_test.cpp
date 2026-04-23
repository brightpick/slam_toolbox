/*
 * Tests for the remapping polygon filter:
 *   - OccupancyGrid::AddScan (filtered overload) / CreateFromScans (filtered)
 *   - SMapper::setRemapping / getOccupancyGrid session routing
 *
 * Suite A  (OccupancyGridFilterTest)  — pure karto, no ROS; exercises ray-clip geometry
 * Suite B  (RemapBboxSMapperTest)     — SMapper; exercises session-based dispatch
 *
 * The tested polygon is always the same axis-aligned rectangle [2,6]×[−1,1]
 * (as a 4-vertex polygon), except the rotated-rectangle test that exercises
 * non-axis-aligned edges.
 *
 * Scan setup: 3-beam LRF covering −5° / 0° / +5°, all at the same range.
 * The central beam (i=1, angle=0 relative to heading) defines the primary
 * endpoint used in assertions.
 *
 * Three identical scans are always added per test to exceed MinPassThrough=2
 * so that cells can be classified as Occupied or Free (not just Unknown).
 *
 * Anchor scan: Suite A tests include a far-away anchor scan at (20, 20) to
 * extend the grid boundary so test scan endpoints never fall at the grid
 * maximum (where WorldToGrid would produce an out-of-bounds index and
 * RayTrace would silently skip the hit — consistent with CreateFromScans).
 */

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "karto_sdk/Karto.h"
#include "karto_sdk/Mapper.h"
#include "slam_toolbox/polygon_fill.hpp"
#include "slam_toolbox/session_label.hpp"
#include "slam_toolbox/slam_mapper.hpp"

namespace
{

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Register a 3-beam custom LRF (−5° / 0° / +5°, range 0.01–100 m).
// Uses override=true so the same name can be safely reused across tests.
void setupLRF(const std::string& name)
{
  auto* lrf = karto::LaserRangeFinder::CreateLaserRangeFinder(
    karto::LaserRangeFinder_Custom, karto::Name(name));
  lrf->SetMinimumAngle(karto::math::DegreesToRadians(-5.0));
  lrf->SetMaximumAngle(karto::math::DegreesToRadians(5.0));
  lrf->SetAngularResolution(karto::math::DegreesToRadians(5.0));
  lrf->SetMinimumRange(0.01);
  lrf->SetMaximumRange(100.0);
  lrf->SetRangeThreshold(100.0);
  karto::SensorManager::GetInstance()->RegisterSensor(lrf, /*override=*/true);
}

// Create a 3-beam scan: sensor at (ox, oy, heading), all beams at 'range'.
// Central beam endpoint = (ox + range*cos(heading), oy + range*sin(heading)).
// Default heading=0 → endpoint at (ox + range, oy).
karto::LocalizedRangeScan* makeScan(int id, double ox, double oy, double range,
                                    const std::string& sensor_name,
                                    double heading = 0.0)
{
  karto::RangeReadingsVector readings(3, range);
  auto* scan = new karto::LocalizedRangeScan(karto::Name(sensor_name), readings);
  scan->SetUniqueId(id);
  scan->SetCorrectedPose(karto::Pose2(ox, oy, heading));
  return scan;
}

// Query occupancy at world point (wx, wy).  Returns GridStates_Unknown if
// the point falls outside the grid.
kt_int8u cellAt(karto::OccupancyGrid* grid, double wx, double wy)
{
  if (!grid) return karto::GridStates_Unknown;
  const auto gp = grid->WorldToGrid(karto::Vector2<kt_double>(wx, wy));
  if (!grid->IsValidGridIndex(gp)) return karto::GridStates_Unknown;
  return *grid->GetDataPointer(gp);
}

// Build an AABB from two corners, as a 4-vertex CCW polygon.
std::vector<karto::Vector2<kt_double>> makeRectPolygon(
    double x1, double y1, double x2, double y2)
{
  return {{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}};
}

// Return a vector with three pointers to the same scan.
// CreateFromScans (filtered) calls AddScan once per entry, so three entries
// gives passCount=6 at the endpoint — enough to exceed MinPassThrough=2.
karto::LocalizedRangeScanVector tripled(karto::LocalizedRangeScan* scan)
{
  return karto::LocalizedRangeScanVector(3, scan);
}

// Session IDs: 0 = base (outside bbox), 1 = remap (inside bbox)
constexpr kt_int32s kBaseSessionId  = 0;
constexpr kt_int32s kRemapSessionId = 1;

// Bbox shared across all geometry tests: x ∈ [2, 6], y ∈ [−1, 1]
constexpr double kBboxX1 = 2.0, kBboxY1 = -1.0;
constexpr double kBboxX2 = 6.0, kBboxY2 =  1.0;
constexpr double kResolution = 0.1;

// Build an ownership image where the given polygon (world coords) is owned
// by kRemapSessionId and everything else by kBaseSessionId.  Delegates the
// rasterization to slam_toolbox::polygon_fill::fillSimplePolygon, the same
// function production buildOwnershipImage uses.
std::unique_ptr<karto::Grid<kt_int32s>> buildTestOwnership(
    const karto::LocalizedRangeScanVector& scans,
    const std::vector<karto::Vector2<kt_double>>& polygonWorld)
{
  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  karto::OccupancyGrid::ComputeDimensions(scans, kResolution, width, height, offset);

  auto grid = std::unique_ptr<karto::Grid<kt_int32s>>(
    karto::Grid<kt_int32s>::CreateGrid(width, height, kResolution));
  grid->GetCoordinateConverter()->SetOffset(offset);

  kt_int32s* data = grid->GetDataPointer();
  const kt_int32s widthStep = grid->GetWidthStep();
  std::fill(data, data + widthStep * height, kBaseSessionId);

  const auto polyGrid = slam_toolbox::polygon_fill::worldToGridPolygon(
    polygonWorld, offset, kResolution);
  slam_toolbox::polygon_fill::fillSimplePolygon<kt_int32s>(
    data, width, height, widthStep, polyGrid, kRemapSessionId);

  return grid;
}

// Session-id lambdas for "allow inside" (scan is remap) and "allow outside" (scan is base)
const auto kAllowInside  = [](karto::LocalizedRangeScan*) -> kt_int32s { return kRemapSessionId; };
const auto kAllowOutside = [](karto::LocalizedRangeScan*) -> kt_int32s { return kBaseSessionId; };

} // namespace


// ═════════════════════════════════════════════════════════════════════════════
// Suite A — OccupancyGrid ray-clip geometry
//
// All tests call CreateFromScans (filtered) directly with a fixed predicate so
// the session layer is bypassed and the geometry is tested in isolation.
//
// Each test uses withAnchor(scan) which appends a far-away anchor scan at
// (20, 20) to the scan vector.  This extends the grid boundary so the test
// scan's endpoint is never at the grid maximum and RayTrace can record hits.
// ═════════════════════════════════════════════════════════════════════════════

class OccupancyGridFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setupLRF(kLaser);
    polygon_ = makeRectPolygon(kBboxX1, kBboxY1, kBboxX2, kBboxY2);
    // Anchor at (20, 20) — far outside the polygon in every test direction.
    // Its bounding box extends the overall grid so test scan endpoints never
    // coincide with the grid maximum (which would cause RayTrace to silently
    // skip the hit, consistent with CreateFromScans behaviour).
    anchor_ = makeScan(kAnchorId, 20.0, 20.0, 0.1, kLaser);
  }

  void TearDown() override
  {
    delete anchor_;
  }

  // Returns tripled(scan) + tripled(anchor_).
  karto::LocalizedRangeScanVector withAnchor(karto::LocalizedRangeScan* scan) const
  {
    auto v = tripled(scan);
    auto a = tripled(anchor_);
    v.insert(v.end(), a.begin(), a.end());
    return v;
  }

  // Build ownership image and call CreateFromScans with it.
  std::unique_ptr<karto::OccupancyGrid> createFiltered(
      const karto::LocalizedRangeScanVector& scans,
      const std::function<kt_int32s(karto::LocalizedRangeScan*)>& fnGetSessionId)
  {
    auto ownership = buildTestOwnership(scans, polygon_);
    return std::unique_ptr<karto::OccupancyGrid>(
      karto::OccupancyGrid::CreateFromScans(scans, kResolution,
                                            ownership.get(), fnGetSessionId));
  }

  // Same as createFiltered but with an arbitrary polygon (rotated, star, ...).
  std::unique_ptr<karto::OccupancyGrid> createFilteredWithPolygon(
      const karto::LocalizedRangeScanVector& scans,
      const std::vector<karto::Vector2<kt_double>>& polygon,
      const std::function<kt_int32s(karto::LocalizedRangeScan*)>& fnGetSessionId)
  {
    auto ownership = buildTestOwnership(scans, polygon);
    return std::unique_ptr<karto::OccupancyGrid>(
      karto::OccupancyGrid::CreateFromScans(scans, kResolution,
                                            ownership.get(), fnGetSessionId));
  }

  const std::string kLaser{"filter_laser"};
  static constexpr int kAnchorId = 999;
  std::vector<karto::Vector2<kt_double>> polygon_;
  karto::LocalizedRangeScan* anchor_{nullptr};
};

// ─── allow_inside / X-axis clipping ──────────────────────────────────────────

// Sensor inside bbox, endpoint inside bbox.
// The entire ray is inside the allowed zone → endpoint Occupied.
TEST_F(OccupancyGridFilterTest, AllowInside_RayFullyInside_EndpointOccupied)
{
  // Sensor (3, 0), range 1.5 → central endpoint (4.5, 0) — both inside box
  auto* scan = makeScan(0, 3.0, 0.0, 1.5, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 4.5, 0.0), karto::GridStates_Occupied);
  delete scan;
}

// Sensor outside bbox, endpoint outside bbox, ray never enters box.
// Nothing is traced → endpoint Unknown.
TEST_F(OccupancyGridFilterTest, AllowInside_RayFullyOutside_NothingTraced)
{
  // Sensor (−2, 0), range 1.0 → central endpoint (−1, 0) — both outside
  auto* scan = makeScan(0, -2.0, 0.0, 1.0, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), -1.0, 0.0), karto::GridStates_Unknown);
  delete scan;
}

// Sensor outside bbox, endpoint inside bbox (ray enters box).
// Only the segment from the bbox entry to the endpoint is traced.
// Endpoint is Occupied; a cell outside the box along the ray is Unknown.
TEST_F(OccupancyGridFilterTest, AllowInside_RayEntersBox_OnlyInsideSegmentTraced)
{
  // Sensor (0, 0), range 3.5 → central endpoint (3.5, 0) — inside box; sensor outside
  auto* scan = makeScan(0, 0.0, 0.0, 3.5, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.5, 0.0), karto::GridStates_Occupied); // inside bbox
  EXPECT_EQ(cellAt(grid.get(), 1.0, 0.0), karto::GridStates_Unknown);  // outside bbox, not traced
  delete scan;
}

// Sensor outside bbox, endpoint outside bbox, ray passes through box.
// Only the inside segment is traced (free space); endpoint is not reached.
// A cell inside the box is Free; the endpoint and cells before the box are Unknown.
TEST_F(OccupancyGridFilterTest, AllowInside_RayPassesThrough_InsideFreeEndpointUnknown)
{
  // Sensor (0, 0), range 8.0 → central endpoint (8, 0) — both outside; crosses [2, 6]
  auto* scan = makeScan(0, 0.0, 0.0, 8.0, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 4.0, 0.0), karto::GridStates_Free);    // inside box, traced as free
  EXPECT_EQ(cellAt(grid.get(), 8.0, 0.0), karto::GridStates_Unknown); // beyond box, not reached
  EXPECT_EQ(cellAt(grid.get(), 1.0, 0.0), karto::GridStates_Unknown); // before box, not reached
  delete scan;
}

// ─── allow_outside / X-axis clipping ─────────────────────────────────────────

// Sensor outside bbox, endpoint outside bbox, ray never enters box.
// Entire ray traced normally → endpoint Occupied.
TEST_F(OccupancyGridFilterTest, AllowOutside_RayFullyOutside_EndpointOccupied)
{
  // Sensor (−2, 0), range 1.0 → central endpoint (−1, 0) — both outside
  auto* scan = makeScan(0, -2.0, 0.0, 1.0, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowOutside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), -1.0, 0.0), karto::GridStates_Occupied);
  delete scan;
}

// Sensor outside bbox, endpoint inside bbox (ray enters box).
// Only the outside segment (origin → bbox entry) is traced.
// Endpoint is Unknown; a cell outside the box along the ray is Free.
TEST_F(OccupancyGridFilterTest, AllowOutside_RayEntersBox_InsideUntouched)
{
  // Sensor (0, 0), range 3.5 → central endpoint (3.5, 0) — inside box
  auto* scan = makeScan(0, 0.0, 0.0, 3.5, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowOutside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.5, 0.0), karto::GridStates_Unknown); // inside bbox, not drawn
  EXPECT_EQ(cellAt(grid.get(), 1.0, 0.0), karto::GridStates_Free);    // outside bbox, traced
  delete scan;
}

// Sensor outside bbox, endpoint outside bbox, ray passes through box.
// Two outside segments traced; inside segment skipped.
// Endpoint Occupied; a cell inside the box is Unknown; cell before box is Free.
TEST_F(OccupancyGridFilterTest, AllowOutside_RayPassesThrough_InsideSkippedEndpointOccupied)
{
  // Sensor (0, 0), range 8.0 → central endpoint (8, 0) — both outside; crosses [2, 6]
  auto* scan = makeScan(0, 0.0, 0.0, 8.0, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowOutside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 8.0, 0.0), karto::GridStates_Occupied); // endpoint outside, traced
  EXPECT_EQ(cellAt(grid.get(), 4.0, 0.0), karto::GridStates_Unknown);  // inside bbox, skipped
  EXPECT_EQ(cellAt(grid.get(), 1.0, 0.0), karto::GridStates_Free);     // outside bbox, before entry
  delete scan;
}

// Sensor inside bbox, endpoint inside bbox.
// Entire ray is inside the forbidden zone → nothing traced.
TEST_F(OccupancyGridFilterTest, AllowOutside_RayFullyInside_NothingTraced)
{
  // Sensor (3, 0), range 1.5 → central endpoint (4.5, 0) — both inside box
  auto* scan = makeScan(0, 3.0, 0.0, 1.5, kLaser);
  auto grid = createFiltered(withAnchor(scan), kAllowOutside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 4.5, 0.0), karto::GridStates_Unknown);
  delete scan;
}

// ─── allow_inside / Y-axis clipping ──────────────────────────────────────────

// Sensor below bbox, heading=π/2 (pointing up), endpoint inside bbox.
// Ray enters through the bottom Y face; endpoint is Occupied.
// Exercises the Y-boundary clip path (distinct from X-axis tests above).
TEST_F(OccupancyGridFilterTest, AllowInside_YAxisRay_EndpointOccupied)
{
  // Sensor (3, −3), heading π/2, range 3.5 → central endpoint (3, 0.5) — inside box
  auto* scan = makeScan(0, 3.0, -3.0, 3.5, kLaser, M_PI / 2.0);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.0, 0.5),  karto::GridStates_Occupied); // endpoint inside bbox
  EXPECT_EQ(cellAt(grid.get(), 3.0, -2.0), karto::GridStates_Unknown);  // below bbox, not traced
  delete scan;
}

// Sensor below bbox, heading=π/2, endpoint above bbox.
// Ray enters through the bottom Y face, exits through the top Y face.
// Only the inside segment is traced → cell inside is Free; endpoint Unknown.
TEST_F(OccupancyGridFilterTest, AllowInside_YAxisRayPassesThrough_InsideFreeEndpointUnknown)
{
  // Sensor (3, −3), heading π/2, range 8.0 → central endpoint (3, 5.0) — outside box
  auto* scan = makeScan(0, 3.0, -3.0, 8.0, kLaser, M_PI / 2.0);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.0,  0.0), karto::GridStates_Free);    // inside bbox, traced as free
  EXPECT_EQ(cellAt(grid.get(), 3.0,  5.0), karto::GridStates_Unknown); // beyond top face, not reached
  EXPECT_EQ(cellAt(grid.get(), 3.0, -2.0), karto::GridStates_Unknown); // below bbox, not traced
  delete scan;
}

// Sensor above bbox, heading=−π/2 (pointing down), endpoint outside bbox.
// Two outside segments traced; inside segment skipped.
// Endpoint Occupied; cell inside bbox Unknown.
TEST_F(OccupancyGridFilterTest, AllowOutside_YAxisRayPassesThrough_InsideSkippedEndpointOccupied)
{
  // Sensor (3, 3), heading −π/2, range 8.0 → central endpoint (3, −5.0) — outside box
  auto* scan = makeScan(0, 3.0, 3.0, 8.0, kLaser, -M_PI / 2.0);
  auto grid = createFiltered(withAnchor(scan), kAllowOutside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.0, -5.0), karto::GridStates_Occupied); // endpoint outside, traced
  EXPECT_EQ(cellAt(grid.get(), 3.0,  0.0), karto::GridStates_Unknown);  // inside bbox, skipped
  EXPECT_EQ(cellAt(grid.get(), 3.0,  2.0), karto::GridStates_Free);     // above bbox, before entry
  delete scan;
}


// ─── diagonal clipping ────────────────────────────────────────────────────────
//
// Sensor at (0, −2), heading=π/4 (45°).  Direction: (1/√2, 1/√2).
//
// Liang-Barsky slab intervals for bbox [2,6]×[−1,1]:
//   X slab: t_enter = (2−0)/(1/√2) = 2√2 ≈ 2.83,  t_exit = 6√2 ≈ 8.49
//   Y slab: t_enter = (−1−(−2))/(1/√2) = √2  ≈ 1.41,  t_exit = 3√2 ≈ 4.24
//
//   t_enter = max(2.83, 1.41) = 2.83  ← X slab wins  → entry point  (2.0,  0.0)
//   t_exit  = min(8.49, 4.24) = 4.24  ← Y slab wins  → exit  point  (3.0,  1.0)
//
// Both slab pairs are active and each determines a different clip boundary.
// A bug in either slab would shift the entry or exit point and break at least
// one of the three per-test assertions.
//
// Key cells:
//   (2.5,  0.5)  — midpoint of clip segment, inside bbox
//   (1.5, −0.5)  — on the ray at t ≈ 2.12, before X entry, outside bbox
//   (5.66, 3.66) — endpoint (≈ 8/√2, −2+8/√2), beyond Y exit, outside bbox

// AllowInside: only the inside clip segment [entry→exit] is drawn.
//   (2.5, 0.5)   → Free    (traced as free space in allowed zone)
//   (5.66, 3.66) → Unknown (beyond Y exit, not reached)
//   (1.5, −0.5)  → Unknown (before X entry, excluded from inside zone)
TEST_F(OccupancyGridFilterTest, AllowInside_DiagonalRay_XEntryYExit_InsideFreeEndpointUnknown)
{
  auto* scan = makeScan(0, 0.0, -2.0, 8.0, kLaser, M_PI / 4.0);
  auto grid = createFiltered(withAnchor(scan), kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(),  2.5,  0.5),  karto::GridStates_Free);    // inside clip segment — free
  EXPECT_EQ(cellAt(grid.get(),  5.66, 3.66), karto::GridStates_Unknown); // beyond Y exit — not reached
  EXPECT_EQ(cellAt(grid.get(),  1.5, -0.5),  karto::GridStates_Unknown); // before X entry — not traced
  delete scan;
}

// AllowOutside: two outside segments are drawn; inside clip segment is skipped.
//   First  outside segment: (0,−2) → entry (2, 0)
//   Inside segment:         (2, 0) → exit  (3, 1)  — SKIPPED
//   Second outside segment: (3, 1) → endpoint (5.66, 3.66)
//
//   (5.66, 3.66) → Occupied (second outside segment endpoint)
//   (2.5,  0.5)  → Unknown  (inside segment, skipped)
//   (1.5, −0.5)  → Free     (first outside segment, before X entry)
TEST_F(OccupancyGridFilterTest, AllowOutside_DiagonalRay_XEntryYExit_InsideSkippedEndpointOccupied)
{
  auto* scan = makeScan(0, 0.0, -2.0, 8.0, kLaser, M_PI / 4.0);
  auto grid = createFiltered(withAnchor(scan), kAllowOutside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(),  5.66,  3.66), karto::GridStates_Occupied); // second outside endpoint
  EXPECT_EQ(cellAt(grid.get(),  2.5,   0.5),  karto::GridStates_Unknown);  // inside segment, skipped
  EXPECT_EQ(cellAt(grid.get(),  1.5,  -0.5),  karto::GridStates_Free);     // first outside segment
  delete scan;
}


// ─── rotated rectangle (non-axis-aligned polygon) ────────────────────────────
//
// Square centred at (4, 0) with side 2, rotated 45° → vertices at
//   (4, √2), (4+√2, 0), (4, −√2), (4−√2, 0)
//   ≈ (4, 1.414), (5.414, 0), (4, −1.414), (2.586, 0)
//
// The rotated square's AABB is [4−√2, −√2] × [4+√2, √2] ≈ [2.586, −1.414]
// to [5.414, 1.414].  Cells inside the AABB but outside the rotated square
// (e.g. near the AABB corners) would be wrongly labelled by an AABB filter
// but correctly excluded by the polygon fill.  This test exercises that
// difference directly.
//
// Reference cells:
//   (4.0, 0.0)   — centre of the square, clearly inside
//   (4.0, 1.3)   — on the vertical axis near the top vertex, inside
//   (5.3, 1.3)   — near the AABB top-right corner; OUTSIDE the rotated square
//                  because it lies beyond the upper-right edge.
TEST_F(OccupancyGridFilterTest, AllowInside_RotatedSquare_OnlyInsidePolygonFilled)
{
  const double s = std::sqrt(2.0);  // ≈ 1.414
  const std::vector<karto::Vector2<kt_double>> rotated{
    {4.0,     s},
    {4.0 + s, 0.0},
    {4.0,    -s},
    {4.0 - s, 0.0}
  };

  // Single sensor inside the polygon at the centre, heading 0, short range.
  // All three beams' endpoints land inside the rotated square.
  auto* scan = makeScan(0, 4.0, 0.0, 0.5, kLaser);
  auto grid = createFilteredWithPolygon(withAnchor(scan), rotated, kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 4.5, 0.0), karto::GridStates_Occupied); // beam endpoint, inside polygon
  // Near the AABB corner (5.3, 1.3) — that lies ~0.4m beyond the upper-right
  // edge of the rotated square (edge: x + y = 4 + √2).  Under AABB filtering
  // (which this test does NOT use) the cell would be "inside"; under polygon
  // filtering it is outside and therefore untouched.
  EXPECT_EQ(cellAt(grid.get(), 5.3, 1.3), karto::GridStates_Unknown);

  delete scan;
}

// ─── non-convex polygon (4-pointed star) ─────────────────────────────────────
//
// A 4-pointed star centred at (4, 0) with outer radius 1.5 and inner radius
// 0.5.  8 vertices alternating between outer and inner.  Scanlines through
// the star produce 4 intersections in general, which the even-odd scanline
// fill handles naturally (4 pairs → 2 filled spans per row).
//
//   outer vertices at 0°/90°/180°/270°: (5.5,0), (4,1.5), (2.5,0), (4,-1.5)
//   inner vertices at 45°/135°/225°/315°: (4+0.5·cos45, 0.5·sin45), ...
TEST_F(OccupancyGridFilterTest, AllowInside_Star_NonConvexFillsInteriorOnly)
{
  const double outer = 1.5;
  const double inner = 0.5;
  const double c = 4.0;  // centre x
  const double s = std::sqrt(0.5);  // 1/√2
  // Ring order: outer(0°), inner(45°), outer(90°), inner(135°),
  //             outer(180°), inner(225°), outer(270°), inner(315°)
  const std::vector<karto::Vector2<kt_double>> star{
    {c + outer,       0.0         },
    {c + inner * s,   inner * s   },
    {c,               outer       },
    {c - inner * s,   inner * s   },
    {c - outer,       0.0         },
    {c - inner * s,  -inner * s   },
    {c,              -outer       },
    {c + inner * s,  -inner * s   }
  };

  // Sensor at the star centre, heading 0, short range — endpoint (4.3, 0)
  // sits inside the star (between centre and +x tip).
  auto* scan = makeScan(0, c, 0.0, 0.3, kLaser);
  auto grid = createFilteredWithPolygon(withAnchor(scan), star, kAllowInside);

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), c + 0.3, 0.0), karto::GridStates_Occupied); // endpoint, inside star
  // Notch region: between two outer tips (e.g. up-right notch near (4.9, 0.9))
  // is OUTSIDE the star (beyond the inner vertex arc) — an AABB / convex
  // hull test would wrongly include it.
  EXPECT_EQ(cellAt(grid.get(), 4.9, 0.9), karto::GridStates_Unknown);

  delete scan;
}

// ─── polygon validation (self-intersection rejected) ─────────────────────────

TEST(SMapperPolygonValidationTest, SelfIntersectingPolygonIsRejected)
{
  mapper_utils::SMapper smapper;
  // Figure-8 (bowtie): edges (0→1) and (2→3) cross.
  std::vector<karto::Vector2<kt_double>> bowtie{
    {0.0, 0.0}, {2.0, 2.0}, {2.0, 0.0}, {0.0, 2.0}
  };
  EXPECT_FALSE(smapper.sessionState().setRemapping(bowtie));
  EXPECT_FALSE(smapper.sessionState().getRemapping().has_value());
}

TEST(SMapperPolygonValidationTest, TooFewVerticesIsRejected)
{
  mapper_utils::SMapper smapper;
  std::vector<karto::Vector2<kt_double>> line{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_FALSE(smapper.sessionState().setRemapping(line));
  EXPECT_FALSE(smapper.sessionState().getRemapping().has_value());
}


// ═════════════════════════════════════════════════════════════════════════════
// Suite B — SMapper session routing
//
// Exercises SMapper::getOccupancyGrid to verify:
//   (a) the remapping guard dispatches correctly to the filtered / unfiltered path
//   (b) session labels determine which region each scan may draw in
// ═════════════════════════════════════════════════════════════════════════════

class RemapBboxSMapperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setupLRF(kLaser);
    smapper_.getMapper()->Initialize(30.0);
    mgr_ = smapper_.getMapper()->GetMapperSensorManager();
    mgr_->RegisterSensor(karto::Name(kLaser));
  }

  // Add three scans at (ox, oy, heading) with the given session and register
  // each scan's post-AddScan UniqueId with the label system.
  //
  // MapperSensorManager::AddScan overwrites pScan->UniqueId with its own
  // monotonic counter, so any id set pre-AddScan is irrelevant.  We read
  // GetUniqueId() after AddScan and register the label under that.
  void addScans(double ox, double oy, double range, int session_id,
                double heading = 0.0)
  {
    slam_toolbox::SessionLabel label;
    label.session_id = session_id;
    smapper_.sessionState().setSessionLabel(label);
    for (int i = 0; i < 3; ++i)
    {
      auto* scan = makeScan(/*id=*/0, ox, oy, range, kLaser, heading);
      mgr_->AddScan(scan);
      smapper_.sessionState().registerNode(scan->GetUniqueId());
      scans_.push_back(scan);
    }
  }

  // Helper: configure remapping with the shared rectangular polygon.
  // Adds two far-apart anchor scans in session 0 to define the grid bounds;
  // setRemapping() then auto-assigns current_session_id = 1 (max of {0} + 1).
  //
  // Rationale: SMapper::getOccupancyGrid sizes the target grid from BASE-
  // SESSION scans only (by design — remap output must slot into the old PGM
  // pixel-for-pixel).  The anchor pair at (-5, -5) and (15, 15) extends the
  // base bbox to cover every assertion point in Suite B without firing rays
  // through the assertion region.  All test scans labelled session 1 must be
  // added AFTER setRemapping — the production flow is: load history →
  // setRemapping → new scans inherit the computed current session.
  void setRemapping()
  {
    addScans(-5.0, -5.0, 0.1, /*session_id=*/0);
    addScans(15.0, 15.0, 0.1, /*session_id=*/0);
    ASSERT_TRUE(smapper_.sessionState().setRemapping(
      makeRectPolygon(kBboxX1, kBboxY1, kBboxX2, kBboxY2)));
    ASSERT_EQ(smapper_.sessionState().getRemapping()->current_session_id, 1);
  }

  void TearDown() override
  {
    for (auto* s : scans_) delete s;
  }

  const std::string kLaser{"smapper_laser"};
  mapper_utils::SMapper smapper_;
  karto::MapperSensorManager* mgr_{nullptr};
  std::vector<karto::LocalizedRangeScan*> scans_;
};

// No remapping configured → unfiltered path: all cells drawn regardless of position.
// Sensor at (3.5, 0) facing left (heading=π) → central endpoint at (0.0, 0.0).
// The endpoint is at the grid minimum (index 0), well within bounds, avoiding
// the off-by-one that occurs when an endpoint coincides with the grid maximum
// in the unfiltered CreateFromScans path (which has no boundary padding).
TEST_F(RemapBboxSMapperTest, NoRemappingConfigured_UnfilteredPath)
{
  addScans(3.5, 0.0, 3.5, /*session_id=*/0, /*heading=*/M_PI);
  // setRemapping not called

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 0.0, 0.0), karto::GridStates_Occupied);
}

// Current remap session (session_id == current_session_id): endpoint inside bbox → Occupied.
TEST_F(RemapBboxSMapperTest, RemappingSession_EndpointInsideBox_IsOccupied)
{
  setRemapping();
  // session_id=1 matches the auto-computed current session; central endpoint (3.5, 0) inside bbox
  addScans(0.0, 0.0, 3.5, /*session_id=*/1);

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.5, 0.0), karto::GridStates_Occupied);
}

// Current remap session: endpoint outside bbox → Unknown (ownership mismatch blocks write).
TEST_F(RemapBboxSMapperTest, RemappingSession_EndpointOutsideBox_IsUnknown)
{
  setRemapping();
  // session_id=1 matches the current session; central endpoint (8, 0) outside bbox
  addScans(0.0, 0.0, 8.0, /*session_id=*/1);

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 8.0, 0.0), karto::GridStates_Unknown);
}

// Base session (session_id=0 ≠ current_session_id): endpoint outside bbox → Occupied.
TEST_F(RemapBboxSMapperTest, FixedSession_EndpointOutsideBox_IsOccupied)
{
  setRemapping();
  // session_id=0 owns everything outside the remap bbox; central endpoint (8, 0) outside
  addScans(0.0, 0.0, 8.0, /*session_id=*/0);

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 8.0, 0.0), karto::GridStates_Occupied);
}

// Base session: endpoint inside bbox → Unknown (ownership mismatch blocks write).
TEST_F(RemapBboxSMapperTest, FixedSession_EndpointInsideBox_IsUnknown)
{
  setRemapping();
  // session_id=0 cannot claim cells inside the remap bbox; central endpoint (3.5, 0) inside
  addScans(0.0, 0.0, 3.5, /*session_id=*/0);

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.5, 0.0), karto::GridStates_Unknown);
}

// Two sessions with non-overlapping rays verify independent spatial control.
//
// Base session   (0): sensor at (0, 2.5), endpoint at (8, 2.5) — entirely above bbox (y=2.5 > 1).
//                     Cells outside bbox are owned by session 0 → traced normally.
// Current remap (1): sensor at (3, 0), endpoint at (4.5, 0) — entirely inside bbox.
//                     Cells inside bbox are owned by session 1 → traced normally.
//
// Assertions:
//   - Base endpoint (8, 2.5) is Occupied   (session 0 drew in its own territory)
//   - Remap endpoint (4.5, 0) is Occupied  (session 1 drew in its own territory)
//   - Cell (4, 0.5) inside bbox but on no ray is Unknown (no cross-contamination)
TEST_F(RemapBboxSMapperTest, TwoSessions_IndependentRegions_NoOverlap)
{
  setRemapping();
  addScans(0.0, 2.5, 8.0, /*session_id=*/0); // base, above bbox
  addScans(3.0, 0.0, 1.5, /*session_id=*/1); // remap, inside bbox

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);

  EXPECT_EQ(cellAt(grid.get(), 8.0, 2.5), karto::GridStates_Occupied); // session 0 drew outside bbox
  EXPECT_EQ(cellAt(grid.get(), 4.5, 0.0), karto::GridStates_Occupied); // session 1 drew inside bbox
  EXPECT_EQ(cellAt(grid.get(), 4.0, 0.5), karto::GridStates_Unknown);  // inside bbox, no ray here
}


// ═════════════════════════════════════════════════════════════════════════════
// Suite C — ownership image layering order
//
// buildOwnershipImage paints session 0 everywhere, then each historical
// session's polygon in ascending session_id order (std::map ordering),
// then the current remapping polygon last.  Higher-id sessions must
// therefore overwrite lower-id sessions in overlapping cells, and the
// current session must overwrite every historical session in cells it
// claims.  OwnershipImage::ownerAtWorld is queried directly — no scans are
// needed because the ownership image is built purely from polygons.
// ═════════════════════════════════════════════════════════════════════════════

TEST(OwnershipLayeringTest, HigherSessionIdOverwritesLowerAndCurrentWinsAll)
{
  mapper_utils::SMapper smapper;

  auto rect = [](double x1, double y1, double x2, double y2) {
    return std::vector<karto::Vector2<kt_double>>{
      {x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}};
  };

  // Configure the current remapping polygon first — node_labels_ is empty
  // at this point, so setRemapping assigns current_session_id = 1.
  ASSERT_TRUE(smapper.sessionState().setRemapping(rect(3.0, 0.0, 5.0, 4.0)));

  // Inject historical labels with two overlapping session polygons.
  // setAllLabels recomputes current_session_id to max(existing) + 1 = 3.
  std::unordered_map<int, slam_toolbox::SessionLabel> labels;
  {
    slam_toolbox::SessionLabel s1;
    s1.session_id = 1;
    s1.polygon = rect(0.0, 0.0, 4.0, 4.0);
    labels[101] = s1;
  }
  {
    slam_toolbox::SessionLabel s2;
    s2.session_id = 2;
    s2.polygon = rect(2.0, 0.0, 6.0, 4.0);
    labels[102] = s2;
  }
  smapper.sessionState().setAllLabels(labels);
  ASSERT_EQ(smapper.sessionState().getRemapping()->current_session_id, 3);

  // World bounds [-2, 10] × [-2, 6] at 1m resolution.  Covers every test
  // point below with room to spare.
  smapper.sessionState().buildOwnershipImage(
    /*width=*/12, /*height=*/8,
    karto::Vector2<kt_double>(-2.0, -2.0),
    /*resolution=*/1.0);

  auto owner = [&](double x, double y) {
    return smapper.sessionState().ownershipImage().ownerAtWorld(karto::Vector2<kt_double>(x, y));
  };

  // Query points land on specific grid cells.  Karto's WorldToGrid rounds
  // half-away-from-zero, not floor; with offset (-2, -2) at resolution 1.0
  // an integer world x=N lands unambiguously on grid cell (N+2).

  // Cell (3, 3) — inside session 1 only.
  EXPECT_EQ(owner(1.0, 1.0), 1);

  // Cell (4, 3) — inside sessions 1 AND 2 (not current).  Higher-id wins.
  EXPECT_EQ(owner(2.0, 1.0), 2);

  // Cell (6, 3) — inside all three (s1, s2, current).  Current overwrites.
  EXPECT_EQ(owner(4.0, 1.0), 3);

  // Cell (8, 3) — inside s2 only (beyond current's right edge at grid 7).
  EXPECT_EQ(owner(6.0, 1.0), 2);

  // Cell (11, 3) — outside every polygon, base session 0 survives.
  EXPECT_EQ(owner(9.0, 1.0), 0);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
