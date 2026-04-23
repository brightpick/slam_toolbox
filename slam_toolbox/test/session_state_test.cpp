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

// ---- SessionState directly ----

TEST(SessionStateTest, DefaultCurrentSessionIdIsZero)
{
  SessionState s;
  s.registerNode(1);
  EXPECT_EQ(s.getSessionId(1), 0);
}

TEST(SessionStateTest, GetSessionIdUnknownNodeReturnsZero)
{
  SessionState s;
  EXPECT_EQ(s.getSessionId(999), 0);
}

TEST(SessionStateTest, SetRemappingTagsSubsequentNodes)
{
  SessionState s;
  ASSERT_TRUE(s.setRemapping(triangle()));
  ASSERT_EQ(s.currentSessionId(), 1);

  s.registerNode(10);
  EXPECT_EQ(s.getSessionId(10), 1);
}

TEST(SessionStateTest, SecondSetRemappingAdvancesSessionId)
{
  SessionState s;

  ASSERT_TRUE(s.setRemapping(triangle()));
  s.registerNode(1);
  s.registerNode(2);

  ASSERT_TRUE(s.setRemapping(triangle()));
  s.registerNode(3);

  EXPECT_EQ(s.getSessionId(1), 1);
  EXPECT_EQ(s.getSessionId(2), 1);
  EXPECT_EQ(s.getSessionId(3), 2);
}

TEST(SessionStateTest, TagNodeAssignsExplicitSessionId)
{
  SessionState s;
  s.tagNode(10, 5);
  s.tagNode(20, 5);

  const auto& all = s.getAllNodeSessionIds();
  EXPECT_EQ(all.size(), 2u);
  EXPECT_EQ(all.at(10), 5);
  EXPECT_EQ(all.at(20), 5);
}

TEST(SessionStateTest, SetAllReplacesBothMaps)
{
  SessionState s;
  s.tagNode(1, 3);

  NodeSessionMap new_nodes{{100, 7}};
  SessionPolygonMap new_polys;
  s.setAll(new_nodes, new_polys);

  EXPECT_EQ(s.getSessionId(1), 0);   // dropped
  EXPECT_EQ(s.getSessionId(100), 7); // new
}

// ---- SMapper wiring ----

TEST(SMapperSessionStateTest, AccessorReturnsUsableState)
{
  mapper_utils::SMapper smapper;
  smapper.sessionState().tagNode(42, 4);

  EXPECT_EQ(smapper.sessionState().getSessionId(42), 4);
  EXPECT_EQ(smapper.sessionState().getSessionId(99), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
