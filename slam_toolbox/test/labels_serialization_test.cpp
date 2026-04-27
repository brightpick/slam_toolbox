#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include "slam_toolbox/remapping/labels_serialization.hpp"

namespace
{

using NodeMap = slam_toolbox::NodeSessionMap;
using PolyMap = slam_toolbox::SessionPolygonMap;
using Polygon = slam_toolbox::Polygon;

class LabelsSerializationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = std::filesystem::temp_directory_path() /
            (std::string("labels_serialization_test_") +
             ::testing::UnitTest::GetInstance()->current_test_info()->name() +
             ".labels");
    std::filesystem::remove(path_);
  }

  void TearDown() override
  {
    std::filesystem::remove(path_);
  }

  std::filesystem::path path_;
};

Polygon triangle(double dx = 0.0)
{
  return {{0.0 + dx, 0.0}, {1.0 + dx, 0.0}, {0.5 + dx, 1.0}};
}

// Self-intersecting "bowtie" — two triangles crossing at the middle.
Polygon bowtie()
{
  return {{0.0, 0.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 1.0}};
}

}  // namespace

// ---- load: missing file ----

TEST_F(LabelsSerializationTest, LoadReturnsFalseWhenFileMissing)
{
  NodeMap nodes;
  PolyMap polygons;
  EXPECT_FALSE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));
  EXPECT_TRUE(nodes.empty());
  EXPECT_TRUE(polygons.empty());
}

// ---- save + load round-trip ----

TEST_F(LabelsSerializationTest, EmptyMapsRoundTripAsEmpty)
{
  slam_toolbox::saveLabels(path_.string(), NodeMap{}, PolyMap{});

  NodeMap nodes;
  PolyMap polygons;
  EXPECT_TRUE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));
  EXPECT_TRUE(nodes.empty());
  EXPECT_TRUE(polygons.empty());
}

TEST_F(LabelsSerializationTest, BaseSessionNodesAreNotEmitted)
{
  // Nodes in session 0 should round-trip as if they were never there —
  // callers treat missing as session 0.
  NodeMap src_nodes{{1, 0}, {2, 0}};
  slam_toolbox::saveLabels(path_.string(), src_nodes, PolyMap{});

  NodeMap nodes;
  PolyMap polygons;
  ASSERT_TRUE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));
  EXPECT_TRUE(nodes.empty());
  EXPECT_TRUE(polygons.empty());
}

TEST_F(LabelsSerializationTest, RoundTripSingleSessionWithPolygon)
{
  NodeMap src_nodes{{10, 1}, {11, 1}};
  PolyMap src_polys{{1, triangle()}};
  slam_toolbox::saveLabels(path_.string(), src_nodes, src_polys);

  NodeMap nodes;
  PolyMap polygons;
  ASSERT_TRUE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));

  ASSERT_EQ(nodes.size(), 2u);
  EXPECT_EQ(nodes[10], 1);
  EXPECT_EQ(nodes[11], 1);

  ASSERT_EQ(polygons.size(), 1u);
  ASSERT_TRUE(polygons.count(1));
  ASSERT_EQ(polygons[1].size(), 3u);
  EXPECT_DOUBLE_EQ(polygons[1][0].GetX(), 0.0);
  EXPECT_DOUBLE_EQ(polygons[1][2].GetY(), 1.0);
}

TEST_F(LabelsSerializationTest, RoundTripMultipleSessionsWithDistinctPolygons)
{
  NodeMap src_nodes{{1, 1}, {2, 2}, {3, 0}};  // pose 3 is base — not emitted
  PolyMap src_polys{{1, triangle(0.0)}, {2, triangle(10.0)}};
  slam_toolbox::saveLabels(path_.string(), src_nodes, src_polys);

  NodeMap nodes;
  PolyMap polygons;
  ASSERT_TRUE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));

  EXPECT_EQ(nodes.count(3), 0u);  // base-session label dropped
  ASSERT_EQ(nodes.size(), 2u);

  ASSERT_EQ(polygons.size(), 2u);
  EXPECT_DOUBLE_EQ(polygons[1][0].GetX(), 0.0);
  EXPECT_DOUBLE_EQ(polygons[2][0].GetX(), 10.0);
}

// ---- load: malformed polygons cause the session to be skipped ----

TEST_F(LabelsSerializationTest, LoadSkipsSessionWithTooFewVertices)
{
  std::ofstream fout(path_);
  fout <<
    "sessions:\n"
    "  - id: 1\n"
    "    polygon: [[0.0, 0.0], [1.0, 0.0]]\n"
    "labels:\n"
    "  - id: 42\n"
    "    session_id: 1\n";
  fout.close();

  NodeMap nodes;
  PolyMap polygons;
  ASSERT_TRUE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));
  // The label still loads, but the malformed session's polygon is dropped.
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes[42], 1);
  EXPECT_EQ(polygons.count(1), 0u);
}

TEST_F(LabelsSerializationTest, LoadSkipsSessionWithSelfIntersectingPolygon)
{
  YAML::Node root;
  YAML::Node session;
  session["id"] = 7;
  YAML::Node poly;
  for (const auto& v : bowtie())
  {
    YAML::Node vertex;
    vertex.push_back(v.GetX());
    vertex.push_back(v.GetY());
    vertex.SetStyle(YAML::EmitterStyle::Flow);
    poly.push_back(vertex);
  }
  session["polygon"] = poly;
  root["sessions"].push_back(session);

  YAML::Node entry;
  entry["id"] = 42;
  entry["session_id"] = 7;
  root["labels"].push_back(entry);

  std::ofstream fout(path_);
  fout << root;
  fout.close();

  NodeMap nodes;
  PolyMap polygons;
  ASSERT_TRUE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(polygons.count(7), 0u);
}

TEST_F(LabelsSerializationTest, LoadReturnsFalseOnCorruptYaml)
{
  std::ofstream fout(path_);
  fout << "sessions: [this is { not valid yaml";
  fout.close();

  NodeMap nodes;
  PolyMap polygons;
  EXPECT_FALSE(slam_toolbox::loadLabels(path_.string(), nodes, polygons));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
