/*
 * Tests for the remapping bounding-box filter:
 *   - OccupancyGrid::AddScan (filtered overload) / CreateFromScans (filtered)
 *   - SMapper::setRemapping / getOccupancyGrid session routing
 *
 * Suite A  (OccupancyGridFilterTest)  — pure karto, no ROS; exercises ray-clip geometry
 * Suite B  (RemapBboxSMapperTest)     — SMapper; exercises session-based dispatch
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

// Build an AABB from two corners.
karto::BoundingBox2 makeBbox(double x1, double y1, double x2, double y2)
{
  karto::BoundingBox2 bb;
  bb.SetMinimum(karto::Vector2<kt_double>(x1, y1));
  bb.SetMaximum(karto::Vector2<kt_double>(x2, y2));
  return bb;
}

// Return a vector with three pointers to the same scan.
// CreateFromScans (filtered) calls AddScan once per entry, so three entries
// gives passCount=6 at the endpoint — enough to exceed MinPassThrough=2.
karto::LocalizedRangeScanVector tripled(karto::LocalizedRangeScan* scan)
{
  return karto::LocalizedRangeScanVector(3, scan);
}

// Always-true / always-false predicates for testing filtered AddScan directly.
const auto kAllowInside  = [](karto::LocalizedRangeScan*) -> kt_bool { return true;  };
const auto kAllowOutside = [](karto::LocalizedRangeScan*) -> kt_bool { return false; };

// Bbox shared across all geometry tests: x ∈ [2, 6], y ∈ [−1, 1]
constexpr double kBboxX1 = 2.0, kBboxY1 = -1.0;
constexpr double kBboxX2 = 6.0, kBboxY2 =  1.0;
constexpr double kResolution = 0.1;

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
    bbox_ = makeBbox(kBboxX1, kBboxY1, kBboxX2, kBboxY2);
    // Anchor at (20, 20) — far outside the bbox in every test direction.
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

  const std::string kLaser{"filter_laser"};
  static constexpr int kAnchorId = 999;
  karto::BoundingBox2 bbox_;
  karto::LocalizedRangeScan* anchor_{nullptr};
};

// ─── allow_inside / X-axis clipping ──────────────────────────────────────────

// Sensor inside bbox, endpoint inside bbox.
// The entire ray is inside the allowed zone → endpoint Occupied.
TEST_F(OccupancyGridFilterTest, AllowInside_RayFullyInside_EndpointOccupied)
{
  // Sensor (3, 0), range 1.5 → central endpoint (4.5, 0) — both inside box
  auto* scan = makeScan(0, 3.0, 0.0, 1.5, kLaser);
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowOutside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowOutside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowOutside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowOutside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowOutside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowInside));

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
  auto grid = std::unique_ptr<karto::OccupancyGrid>(
    karto::OccupancyGrid::CreateFromScans(withAnchor(scan), kResolution, &bbox_, kAllowOutside));

  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(),  5.66,  3.66), karto::GridStates_Occupied); // second outside endpoint
  EXPECT_EQ(cellAt(grid.get(),  2.5,   0.5),  karto::GridStates_Unknown);  // inside segment, skipped
  EXPECT_EQ(cellAt(grid.get(),  1.5,  -0.5),  karto::GridStates_Free);     // first outside segment
  delete scan;
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

  // Add three scans at (ox, oy, heading) with the given session and register their
  // unique IDs with the smapper label system.
  // base_id must not overlap across addScans() calls within a single test.
  void addScans(double ox, double oy, double range, int session_id,
                int base_id = 0, double heading = 0.0)
  {
    slam_toolbox::SessionLabel label;
    label.session_id = session_id;
    smapper_.setSessionLabel(label);
    for (int i = 0; i < 3; ++i)
    {
      const int id = base_id + i;
      auto* scan = makeScan(id, ox, oy, range, kLaser, heading);
      mgr_->AddScan(scan);
      smapper_.registerNode(id);
      scans_.push_back(scan);
    }
  }

  // Helper: configure remapping with the shared bbox and the given session IDs.
  // Also adds an anchor scan in a far-away fixed session (session_id=kAnchorSessionId)
  // to extend the grid boundary, ensuring test scan endpoints never fall at the
  // grid maximum (same boundary issue as Suite A — CreateFromScans (filtered) has no
  // built-in padding, consistent with CreateFromScans).
  void setRemapping(std::unordered_set<int> ids)
  {
    // Anchor: session kAnchorSessionId is never in non_fixed_session_ids, so it is
    // always treated as a fixed session and may only draw outside the bbox.
    // Its endpoint at (20.1, 20) is outside bbox [2,6]×[−1,1] → no interference.
    addScans(20.0, 20.0, 0.1, /*session_id=*/kAnchorSessionId, /*base_id=*/kAnchorBaseId);

    mapper_utils::SMapper::RemappingConfig cfg;
    cfg.non_fixed_session_ids = std::move(ids);
    cfg.bbox = makeBbox(kBboxX1, kBboxY1, kBboxX2, kBboxY2);
    smapper_.setRemapping(std::move(cfg));
  }

  void TearDown() override
  {
    for (auto* s : scans_) delete s;
  }

  const std::string kLaser{"smapper_laser"};
  // Session ID and base scan ID reserved for the grid-boundary anchor scan.
  // This session is never included in non_fixed_session_ids so it is always fixed.
  static constexpr int kAnchorSessionId = 255;
  static constexpr int kAnchorBaseId    = 900;
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
  addScans(3.5, 0.0, 3.5, /*session_id=*/0, /*base_id=*/0, /*heading=*/M_PI);
  // setRemapping not called

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 0.0, 0.0), karto::GridStates_Occupied);
}

// Remapping session (session_id in non_fixed_session_ids): endpoint inside bbox → Occupied.
TEST_F(RemapBboxSMapperTest, RemappingSession_EndpointInsideBox_IsOccupied)
{
  // session_id=1 is remapping; central endpoint (3.5, 0) inside bbox
  addScans(0.0, 0.0, 3.5, /*session_id=*/1);
  setRemapping({1});

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.5, 0.0), karto::GridStates_Occupied);
}

// Remapping session: endpoint outside bbox → Unknown (filter blocks drawing outside).
TEST_F(RemapBboxSMapperTest, RemappingSession_EndpointOutsideBox_IsUnknown)
{
  // session_id=1 is remapping; central endpoint (8, 0) outside bbox
  addScans(0.0, 0.0, 8.0, /*session_id=*/1);
  setRemapping({1});

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 8.0, 0.0), karto::GridStates_Unknown);
}

// Fixed session (session_id NOT in non_fixed_session_ids): endpoint outside bbox → Occupied.
TEST_F(RemapBboxSMapperTest, FixedSession_EndpointOutsideBox_IsOccupied)
{
  // session_id=0 is fixed; central endpoint (8, 0) outside bbox
  addScans(0.0, 0.0, 8.0, /*session_id=*/0);
  setRemapping({1}); // session 0 is fixed

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 8.0, 0.0), karto::GridStates_Occupied);
}

// Fixed session: endpoint inside bbox → Unknown (filter blocks drawing inside).
TEST_F(RemapBboxSMapperTest, FixedSession_EndpointInsideBox_IsUnknown)
{
  // session_id=0 is fixed; central endpoint (3.5, 0) inside bbox
  addScans(0.0, 0.0, 3.5, /*session_id=*/0);
  setRemapping({1});

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);
  EXPECT_EQ(cellAt(grid.get(), 3.5, 0.0), karto::GridStates_Unknown);
}

// Two sessions with non-overlapping rays verify independent spatial control.
//
// Fixed session   (0): sensor at (0, 2.5), endpoint at (8, 2.5) — entirely above bbox (y=2.5 > 1).
//                       Ray never enters bbox → traced normally.
// Remapping session (1): sensor at (3, 0),  endpoint at (4.5, 0) — entirely inside bbox.
//                       Sensor inside bbox  → traced normally in allowed zone.
//
// Assertions:
//   - Fixed endpoint (8, 2.5) is Occupied   (fixed drew outside bbox)
//   - Remapping endpoint (4.5, 0) is Occupied (remapping drew inside bbox)
//   - Cell (4, 0.5) inside bbox but on no ray is Unknown (no cross-contamination)
TEST_F(RemapBboxSMapperTest, TwoSessions_IndependentRegions_NoOverlap)
{
  addScans(0.0, 2.5, 8.0, /*session_id=*/0, /*base_id=*/0); // fixed, above bbox
  addScans(3.0, 0.0, 1.5, /*session_id=*/1, /*base_id=*/3); // remapping, inside bbox

  setRemapping({1});

  auto grid = std::unique_ptr<karto::OccupancyGrid>(smapper_.getOccupancyGrid(kResolution));
  ASSERT_NE(grid, nullptr);

  EXPECT_EQ(cellAt(grid.get(), 8.0, 2.5), karto::GridStates_Occupied); // fixed drew outside bbox
  EXPECT_EQ(cellAt(grid.get(), 4.5, 0.0), karto::GridStates_Occupied); // remapping drew inside bbox
  EXPECT_EQ(cellAt(grid.get(), 4.0, 0.5), karto::GridStates_Unknown);  // inside bbox, no ray here
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
