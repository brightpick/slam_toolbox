#include <gtest/gtest.h>

#include <unordered_map>

#include "slam_toolbox/session_state.hpp"
#include "slam_toolbox/slam_mapper.hpp"

using slam_toolbox::NodeSessionMap;
using slam_toolbox::Polygon;
using slam_toolbox::SessionPolygonMap;
using slam_toolbox::SessionState;

namespace
{

Polygon triangle()
{
  return {{0.0, 0.0}, {1.0, 0.0}, {0.5, 1.0}};
}

}  // namespace

// All assertions go through the public surface that production uses:
// getAllNodeSessionIds() (read by the labels-serialization path) and the
// predicate factories (consumed by the solver / loop closure / grid
// hooks).  Internal fields like the per-node session id and the
// "current" session id are deliberately not poked at directly.

TEST(SessionStateTest, RegisterWithoutRemappingTagsBaseSession)
{
  SessionState s;
  s.registerNode(1);
  EXPECT_EQ(s.getAllNodeSessionIds().at(1), slam_toolbox::kBaseSessionId);
}

TEST(SessionStateTest, SetRemappingTagsSubsequentRegistrations)
{
  SessionState s;
  s.registerNode(1);                         // pre-remap → base
  ASSERT_TRUE(s.setRemapping(triangle()));
  s.registerNode(2);                         // post-remap → current

  const auto& nodes = s.getAllNodeSessionIds();
  EXPECT_EQ(nodes.at(1), slam_toolbox::kBaseSessionId);
  EXPECT_NE(nodes.at(2), slam_toolbox::kBaseSessionId);
  EXPECT_NE(nodes.at(2), nodes.at(1));

  // Observable through the predicate the solver consumes.
  auto pinned = s.makeFixedPosePredicate();
  EXPECT_TRUE(pinned(1));    // base node — would be held fixed
  EXPECT_FALSE(pinned(2));   // current-session node — free to move
  EXPECT_TRUE(pinned(999));  // unknown node defaults to base
}

TEST(SessionStateTest, SecondSetRemappingAdvancesSession)
{
  SessionState s;

  ASSERT_TRUE(s.setRemapping(triangle()));
  s.registerNode(1);
  s.registerNode(2);

  ASSERT_TRUE(s.setRemapping(triangle()));
  s.registerNode(3);

  const auto& nodes = s.getAllNodeSessionIds();
  EXPECT_EQ(nodes.at(1), nodes.at(2));   // 1 & 2 share a session
  EXPECT_NE(nodes.at(1), nodes.at(3));   // 3 is in a different one

  // Both polygons are recorded for serialization.
  EXPECT_EQ(s.getAllSessionPolygons().size(), 2u);
}

TEST(SessionStateTest, SetRemappingRejectsInvalidPolygon)
{
  SessionState s;
  Polygon bowtie{{0.0, 0.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 1.0}};
  EXPECT_FALSE(s.setRemapping(bowtie));
  EXPECT_FALSE(s.getRemappingPolygon().has_value());
}

TEST(SessionStateTest, ConstructFromLoadedHistory)
{
  NodeSessionMap loaded_nodes{{100, 7}};
  SessionPolygonMap loaded_polys{{7, triangle()}};
  SessionState s{loaded_nodes, loaded_polys};

  const auto& nodes = s.getAllNodeSessionIds();
  EXPECT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.at(100), 7);

  EXPECT_EQ(s.getAllSessionPolygons().size(), 1u);
}

TEST(SessionStateTest, SetRemappingAfterLoadAvoidsSessionIdCollision)
{
  // Production flow: deserialize completes (loads history with session ids
  // 1 and 2), then user calls start_remapping — setRemapping must pick a
  // fresh id that doesn't collide with the loaded history.
  NodeSessionMap loaded_nodes{{101, 1}, {102, 2}};
  SessionPolygonMap loaded_polys{
    {1, triangle()},
    {2, triangle()}};
  SessionState s{loaded_nodes, loaded_polys};

  ASSERT_TRUE(s.setRemapping(triangle()));
  s.registerNode(200);
  const auto& nodes = s.getAllNodeSessionIds();
  const int new_sid = nodes.at(200);
  EXPECT_NE(new_sid, 1);
  EXPECT_NE(new_sid, 2);
  EXPECT_NE(new_sid, slam_toolbox::kBaseSessionId);
}

// ---- SMapper wiring ----

TEST(SMapperSessionStateTest, AccessorReturnsUsableState)
{
  mapper_utils::SMapper smapper;
  ASSERT_TRUE(smapper.sessionState().setRemapping(triangle()));
  smapper.sessionState().registerNode(42);

  const auto& nodes = smapper.sessionState().getAllNodeSessionIds();
  ASSERT_EQ(nodes.count(42), 1u);
  EXPECT_NE(nodes.at(42), slam_toolbox::kBaseSessionId);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
