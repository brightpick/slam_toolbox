#include <gtest/gtest.h>

#include "slam_toolbox/remapping/polygon_units.hpp"

using slam_toolbox::Polygon;
using slam_toolbox::pixelPolygonToWorld;

namespace
{

constexpr double kTol = 1e-9;

}  // namespace

TEST(PolygonUnitsTest, ZeroOffsetUnitResolution_VerticalFlipMatches)
{
  // height=10 → grid row for py=0 is (10-1-0) = 9; for py=9 is 0.
  Polygon pixel{{2.0, 0.0}, {7.0, 9.0}};
  auto world = pixelPolygonToWorld(pixel,
                            karto::Vector2<kt_double>(0.0, 0.0),
                            /*resolution=*/1.0,
                            /*height=*/10);
  ASSERT_EQ(world.size(), 2u);
  EXPECT_NEAR(world[0].GetX(), 2.0, kTol);
  EXPECT_NEAR(world[0].GetY(), 9.0, kTol);
  EXPECT_NEAR(world[1].GetX(), 7.0, kTol);
  EXPECT_NEAR(world[1].GetY(), 0.0, kTol);
}

TEST(PolygonUnitsTest, NonZeroOffsetIsAdded)
{
  Polygon pixel{{0.0, 0.0}};
  auto world = pixelPolygonToWorld(pixel,
                            karto::Vector2<kt_double>(-2.0, -3.0),
                            /*resolution=*/1.0,
                            /*height=*/4);
  ASSERT_EQ(world.size(), 1u);
  EXPECT_NEAR(world[0].GetX(), -2.0, kTol);
  EXPECT_NEAR(world[0].GetY(), -3.0 + (4 - 1) * 1.0, kTol);  // -3 + 3 = 0
}

TEST(PolygonUnitsTest, ResolutionScales)
{
  Polygon pixel{{4.0, 2.0}};
  auto world = pixelPolygonToWorld(pixel,
                            karto::Vector2<kt_double>(0.0, 0.0),
                            /*resolution=*/0.05,
                            /*height=*/100);
  ASSERT_EQ(world.size(), 1u);
  EXPECT_NEAR(world[0].GetX(), 0.20, kTol);
  EXPECT_NEAR(world[0].GetY(), (100 - 1 - 2) * 0.05, kTol);  // 4.85
}

TEST(PolygonUnitsTest, EmptyInputProducesEmptyOutput)
{
  Polygon pixel;
  auto world = pixelPolygonToWorld(pixel,
                            karto::Vector2<kt_double>(0.0, 0.0),
                            /*resolution=*/0.05,
                            /*height=*/100);
  EXPECT_TRUE(world.empty());
}

TEST(PolygonUnitsTest, FractionalPixelCoords)
{
  // Pixel coords don't need to be integers — ROS Polygon uses float32 points.
  Polygon pixel{{1.5, 2.5}};
  auto world = pixelPolygonToWorld(pixel,
                            karto::Vector2<kt_double>(0.0, 0.0),
                            /*resolution=*/0.10,
                            /*height=*/10);
  ASSERT_EQ(world.size(), 1u);
  EXPECT_NEAR(world[0].GetX(), 0.15, kTol);
  EXPECT_NEAR(world[0].GetY(), (10 - 1 - 2.5) * 0.10, kTol);  // 0.65
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
