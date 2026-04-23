#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <unordered_map>

#include "slam_toolbox/labels_serialization.hpp"

namespace
{

using slam_toolbox::SessionLabel;
using LabelMap = std::unordered_map<int, SessionLabel>;
namespace LS = slam_toolbox::labels_serialization;

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

SessionLabel makeLabel(int session_id)
{
  SessionLabel label;
  label.session_id = session_id;
  return label;
}

SessionLabel makeLabelWithPolygon(int session_id,
  std::vector<karto::Vector2<kt_double>> polygon)
{
  SessionLabel label;
  label.session_id = session_id;
  label.polygon = std::move(polygon);
  return label;
}

// A simple (non-crossing) triangle.
std::vector<karto::Vector2<kt_double>> triangle(double dx = 0.0)
{
  return {{0.0 + dx, 0.0}, {1.0 + dx, 0.0}, {0.5 + dx, 1.0}};
}

// A self-intersecting "bowtie" — two triangles crossing at the middle.
std::vector<karto::Vector2<kt_double>> bowtie()
{
  return {{0.0, 0.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 1.0}};
}

}  // namespace

// ---- load: missing file ----

TEST_F(LabelsSerializationTest, LoadReturnsFalseWhenFileMissing)
{
  LabelMap labels;
  EXPECT_FALSE(LS::load(path_.string(), labels));
  EXPECT_TRUE(labels.empty());
}

// ---- save+load round-trip ----

TEST_F(LabelsSerializationTest, EmptyMapRoundTripsAsEmpty)
{
  LabelMap src;
  LS::save(path_.string(), src);

  LabelMap dst;
  EXPECT_TRUE(LS::load(path_.string(), dst));
  EXPECT_TRUE(dst.empty());
}

TEST_F(LabelsSerializationTest, BaseSessionLabelsAreNotEmitted)
{
  // Labels in session 0 should round-trip as if they were never there —
  // callers treat missing as session 0.
  LabelMap src;
  src[1] = makeLabel(0);
  src[2] = makeLabel(0);
  LS::save(path_.string(), src);

  LabelMap dst;
  ASSERT_TRUE(LS::load(path_.string(), dst));
  EXPECT_TRUE(dst.empty());
}

TEST_F(LabelsSerializationTest, RoundTripSingleSessionWithPolygon)
{
  LabelMap src;
  src[10] = makeLabelWithPolygon(1, triangle());
  src[11] = makeLabelWithPolygon(1, triangle());
  LS::save(path_.string(), src);

  LabelMap dst;
  ASSERT_TRUE(LS::load(path_.string(), dst));

  ASSERT_EQ(dst.size(), 2u);
  ASSERT_TRUE(dst.count(10));
  ASSERT_TRUE(dst.count(11));
  EXPECT_EQ(dst[10].session_id, 1);
  EXPECT_EQ(dst[11].session_id, 1);

  ASSERT_TRUE(dst[10].polygon.has_value());
  ASSERT_TRUE(dst[11].polygon.has_value());
  ASSERT_EQ(dst[10].polygon->size(), 3u);
  EXPECT_DOUBLE_EQ((*dst[10].polygon)[0].GetX(), 0.0);
  EXPECT_DOUBLE_EQ((*dst[10].polygon)[2].GetY(), 1.0);
}

TEST_F(LabelsSerializationTest, RoundTripMultipleSessionsWithDistinctPolygons)
{
  LabelMap src;
  src[1] = makeLabelWithPolygon(1, triangle(0.0));
  src[2] = makeLabelWithPolygon(2, triangle(10.0));
  src[3] = makeLabel(0);  // base — not emitted
  LS::save(path_.string(), src);

  LabelMap dst;
  ASSERT_TRUE(LS::load(path_.string(), dst));

  // Base-session label 3 is dropped on save.
  EXPECT_EQ(dst.count(3), 0u);
  ASSERT_EQ(dst.size(), 2u);

  ASSERT_TRUE(dst[1].polygon.has_value());
  ASSERT_TRUE(dst[2].polygon.has_value());
  EXPECT_DOUBLE_EQ((*dst[1].polygon)[0].GetX(), 0.0);
  EXPECT_DOUBLE_EQ((*dst[2].polygon)[0].GetX(), 10.0);
}

TEST_F(LabelsSerializationTest, LabelInheritsPolygonFromSessionEntry)
{
  // Only one label per session carries the polygon on save, but every label
  // for that session must re-acquire it on load.
  LabelMap src;
  src[100] = makeLabelWithPolygon(5, triangle());
  src[101] = makeLabel(5);  // same session, no polygon on the struct
  LS::save(path_.string(), src);

  LabelMap dst;
  ASSERT_TRUE(LS::load(path_.string(), dst));
  ASSERT_EQ(dst.size(), 2u);
  ASSERT_TRUE(dst[100].polygon.has_value());
  ASSERT_TRUE(dst[101].polygon.has_value());
  EXPECT_EQ(dst[100].polygon->size(), 3u);
  EXPECT_EQ(dst[101].polygon->size(), 3u);
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

  LabelMap dst;
  ASSERT_TRUE(LS::load(path_.string(), dst));
  // The label is still loaded, but without a polygon since the session was
  // skipped — the join fails silently for that session_id.
  ASSERT_EQ(dst.size(), 1u);
  EXPECT_EQ(dst[42].session_id, 1);
  EXPECT_FALSE(dst[42].polygon.has_value());
}

TEST_F(LabelsSerializationTest, LoadSkipsSessionWithSelfIntersectingPolygon)
{
  LabelMap src;
  // Can't use `save` to write a bad polygon — the save path doesn't validate
  // (the invariant is enforced by SMapper::setRemapping).  Write by hand.
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

  LabelMap dst;
  ASSERT_TRUE(LS::load(path_.string(), dst));
  ASSERT_EQ(dst.size(), 1u);
  EXPECT_FALSE(dst[42].polygon.has_value());
}

TEST_F(LabelsSerializationTest, LoadReturnsFalseOnCorruptYaml)
{
  std::ofstream fout(path_);
  fout << "sessions: [this is { not valid yaml";
  fout.close();

  LabelMap dst;
  EXPECT_FALSE(LS::load(path_.string(), dst));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
