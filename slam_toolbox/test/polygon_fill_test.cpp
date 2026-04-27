/*
 * Direct unit tests for the polygon_fill helpers.
 *
 * The function is exercised indirectly through the ownership-image tests in
 * remap_bbox_test.cpp, but that's through the SMapper + scan machinery.
 * These tests call fillSimplePolygon / isSimplePolygon on a plain buffer so
 * failures point straight at the algorithm instead of the surrounding ROS
 * plumbing.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "slam_toolbox/remapping/polygon_fill.hpp"

using slam_toolbox::fillSimplePolygon;
using slam_toolbox::isSimplePolygon;
using slam_toolbox::worldToGridPolygon;
using Vec = karto::Vector2<kt_double>;

namespace
{

// Allocate a width x height grid, filled with `bg`, with rows strided by
// `widthStep`.  widthStep may exceed width to mimic karto::Grid's 8-byte
// alignment padding.
struct TestGrid
{
  std::vector<kt_int32s> data;
  kt_int32s width, height, widthStep;

  TestGrid(kt_int32s w, kt_int32s h, kt_int32s ws, kt_int32s bg)
    : data(static_cast<std::size_t>(ws) * static_cast<std::size_t>(h), bg),
      width(w), height(h), widthStep(ws) {}

  kt_int32s at(kt_int32s x, kt_int32s y) const
  {
    return data[static_cast<std::size_t>(y) * static_cast<std::size_t>(widthStep)
              + static_cast<std::size_t>(x)];
  }
};

} // namespace


// ─── isSimplePolygon ────────────────────────────────────────────────────────

TEST(IsSimplePolygonTest, RejectsFewerThan3Vertices)
{
  EXPECT_FALSE(isSimplePolygon({}));
  EXPECT_FALSE(isSimplePolygon({Vec(0, 0)}));
  EXPECT_FALSE(isSimplePolygon({Vec(0, 0), Vec(1, 1)}));
}

TEST(IsSimplePolygonTest, AcceptsConvexTriangle)
{
  EXPECT_TRUE(isSimplePolygon({Vec(0, 0), Vec(2, 0), Vec(1, 2)}));
}

TEST(IsSimplePolygonTest, AcceptsConvexSquareCCW)
{
  EXPECT_TRUE(isSimplePolygon(
    {Vec(0, 0), Vec(4, 0), Vec(4, 4), Vec(0, 4)}));
}

TEST(IsSimplePolygonTest, AcceptsConvexSquareCW)
{
  // Winding order shouldn't matter — both CCW and CW are simple.
  EXPECT_TRUE(isSimplePolygon(
    {Vec(0, 0), Vec(0, 4), Vec(4, 4), Vec(4, 0)}));
}

TEST(IsSimplePolygonTest, AcceptsNonConvexStar)
{
  // 4-pointed star: outer/inner vertices alternating on the compass.
  EXPECT_TRUE(isSimplePolygon({
    Vec(2, 0), Vec(0.5, 0.5),
    Vec(0, 2), Vec(-0.5, 0.5),
    Vec(-2, 0), Vec(-0.5, -0.5),
    Vec(0, -2), Vec(0.5, -0.5)
  }));
}

TEST(IsSimplePolygonTest, RejectsBowtie)
{
  // Classic figure-8: edges (0→1) and (2→3) cross through the interior.
  EXPECT_FALSE(isSimplePolygon(
    {Vec(0, 0), Vec(2, 2), Vec(2, 0), Vec(0, 2)}));
}


// ─── fillSimplePolygon ───────────────────────────────────────────────────────
//
// These tests keep the grid small (no byte-alignment padding) so cells can
// be reasoned about directly.  The WidthStep-vs-Width mismatch is covered
// by WidthStepDifferentFromWidth further down.

TEST(FillSimplePolygonTest, AxisAlignedRectangleFillsExactly)
{
  // Grid 6x4.  Rectangle vertices (1,1)-(5,1)-(5,3)-(1,3).
  // Scanline fill is inclusive in x (ceil(xs_lo) .. floor(xs_hi)) and
  // half-open in y, so for integer-aligned vertices:
  //   y=1,2 → x=1..5 (5 cells each).  y=3 excluded by half-open rule.
  TestGrid grid(6, 4, 6, 0);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    {Vec(1, 1), Vec(5, 1), Vec(5, 3), Vec(1, 3)}, 7);

  for (kt_int32s y = 0; y < 4; ++y)
  {
    for (kt_int32s x = 0; x < 6; ++x)
    {
      const bool inside = (x >= 1 && x <= 5 && y >= 1 && y <= 2);
      EXPECT_EQ(grid.at(x, y), inside ? 7 : 0)
        << " at cell (" << x << "," << y << ")";
    }
  }
}

TEST(FillSimplePolygonTest, TriangleFillsExpectedCells)
{
  // Triangle with vertices (0,0), (4,0), (0,4).  The bottom edge is
  // horizontal and skipped by the scanline fill; the per-cell assertions
  // below are the spec for what the half-open edge rule must fill.
  TestGrid grid(6, 6, 6, 0);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    {Vec(0, 0), Vec(4, 0), Vec(0, 4)}, 1);

  // Interior cells — clearly inside the triangle.
  EXPECT_EQ(grid.at(0, 0), 1);
  EXPECT_EQ(grid.at(1, 1), 1);
  EXPECT_EQ(grid.at(0, 3), 1);
  EXPECT_EQ(grid.at(3, 0), 1);

  // Outside (beyond the hypotenuse).
  EXPECT_EQ(grid.at(3, 2), 0);
  EXPECT_EQ(grid.at(4, 1), 0);
  EXPECT_EQ(grid.at(5, 5), 0);
}

TEST(FillSimplePolygonTest, NonConvexLShape)
{
  // L-shape polygon vertices: (0,0) (3,0) (3,2) (2,2) (2,4) (0,4).
  // Scanline fill is inclusive on the right edge (x=floor(xs_hi)):
  //   y=0,1 → xs=[0,3]  → x=0..3 (4 cells)   — wide top
  //   y=2,3 → xs=[0,2]  → x=0..2 (3 cells)   — narrow bottom
  //   y=4   → excluded by half-open rule
  TestGrid grid(4, 5, 4, 0);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    {Vec(0, 0), Vec(3, 0), Vec(3, 2), Vec(2, 2), Vec(2, 4), Vec(0, 4)}, 9);

  auto inside = [](kt_int32s x, kt_int32s y)
  {
    if (y >= 0 && y <= 1) return x >= 0 && x <= 3;
    if (y >= 2 && y <= 3) return x >= 0 && x <= 2;
    return false;
  };

  for (kt_int32s y = 0; y < 5; ++y)
  {
    for (kt_int32s x = 0; x < 4; ++x)
    {
      EXPECT_EQ(grid.at(x, y), inside(x, y) ? 9 : 0)
        << " at cell (" << x << "," << y << ")";
    }
  }
}

TEST(FillSimplePolygonTest, NonConvex4PointedStarProducesTwoSpansPerRow)
{
  // 4-pointed star centred at (5, 5), outer radius 4, inner radius 1.
  // Some scanlines hit 4 edges — two filled spans per row, with a gap
  // at the centre where the scanline passes through the concavity.
  const double c = 5.0;
  const double outer = 4.0;
  const double inner = 1.0;
  const double s = 1.0 / std::sqrt(2.0);

  std::vector<Vec> star{
    {c + outer,      c          },
    {c + inner * s,  c + inner * s},
    {c,              c + outer  },
    {c - inner * s,  c + inner * s},
    {c - outer,      c          },
    {c - inner * s,  c - inner * s},
    {c,              c - outer  },
    {c + inner * s,  c - inner * s}
  };

  TestGrid grid(11, 11, 16 /* widthStep padded */, 0);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    star, 3);

  // Along the +x arm: cells (7,5) and (8,5) are inside (outer tip).
  EXPECT_EQ(grid.at(7, 5), 3);
  EXPECT_EQ(grid.at(8, 5), 3);

  // Notch between the +x and +y arms (around (8, 8)) — outside the star.
  EXPECT_EQ(grid.at(8, 8), 0);
  EXPECT_EQ(grid.at(9, 8), 0);

  // Centre is inside (inner ring covers it).
  EXPECT_EQ(grid.at(5, 5), 3);
}

TEST(FillSimplePolygonTest, WidthStepDifferentFromWidth)
{
  // Grid with width=5 and widthStep=8 (aligned-up padding).  Filling a
  // rectangle must index rows by widthStep, otherwise writes land in
  // neighbouring row memory and the test assertions via widthStep indexing
  // will see zeros where the fill should have landed.
  TestGrid grid(5, 3, 8, 0);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    {Vec(0, 0), Vec(5, 0), Vec(5, 3), Vec(0, 3)}, 4);

  for (kt_int32s y = 0; y < 3; ++y)
  {
    for (kt_int32s x = 0; x < 5; ++x)
    {
      EXPECT_EQ(grid.at(x, y), 4)
        << " at cell (" << x << "," << y << ")";
    }
    // Padding bytes (x = 5..7) must remain the background value.
    for (kt_int32s x = 5; x < 8; ++x)
    {
      EXPECT_EQ(grid.at(x, y), 0)
        << " padding cell (" << x << "," << y << ") was overwritten";
    }
  }
}

TEST(FillSimplePolygonTest, PolygonPartlyOutsideGridClipsToBounds)
{
  // 10x10 rectangle clipped against a 5x5 grid.
  TestGrid grid(5, 5, 5, 0);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    {Vec(-3, -3), Vec(8, -3), Vec(8, 8), Vec(-3, 8)}, 1);

  for (kt_int32s y = 0; y < 5; ++y)
  {
    for (kt_int32s x = 0; x < 5; ++x)
    {
      EXPECT_EQ(grid.at(x, y), 1)
        << " at cell (" << x << "," << y << ")";
    }
  }
}

TEST(FillSimplePolygonTest, DegeneratePolygonIsNoop)
{
  TestGrid grid(3, 3, 3, 7);
  fillSimplePolygon<kt_int32s>(
    grid.data.data(), grid.width, grid.height, grid.widthStep,
    {Vec(0, 0), Vec(1, 1)}, 9);
  // Only 2 vertices → silently skipped; grid unchanged.
  for (auto v : grid.data) EXPECT_EQ(v, 7);
}


// ─── worldToGridPolygon ─────────────────────────────────────────────────────

TEST(WorldToGridPolygonTest, ConvertsVerticesUsingOffsetAndResolution)
{
  const Vec offset(1.0, 2.0);
  const double resolution = 0.5;
  const std::vector<Vec> world{Vec(1.0, 2.0), Vec(2.0, 2.0), Vec(2.0, 3.0)};

  const auto grid = worldToGridPolygon(world, offset, resolution);
  ASSERT_EQ(grid.size(), 3u);
  EXPECT_DOUBLE_EQ(grid[0].GetX(), 0.0);
  EXPECT_DOUBLE_EQ(grid[0].GetY(), 0.0);
  EXPECT_DOUBLE_EQ(grid[1].GetX(), 2.0);
  EXPECT_DOUBLE_EQ(grid[1].GetY(), 0.0);
  EXPECT_DOUBLE_EQ(grid[2].GetX(), 2.0);
  EXPECT_DOUBLE_EQ(grid[2].GetY(), 2.0);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
