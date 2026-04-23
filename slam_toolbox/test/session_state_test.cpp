#include <gtest/gtest.h>

#include <unordered_map>

#include "slam_toolbox/session_state.hpp"
#include "slam_toolbox/slam_mapper.hpp"

using slam_toolbox::SessionState;

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

TEST(SessionStateTest, SetCurrentSessionIdTagsSubsequentNodes)
{
  SessionState s;
  s.setCurrentSessionId(3);
  s.registerNode(10);

  EXPECT_EQ(s.getSessionId(10), 3);
}

TEST(SessionStateTest, SetCurrentSessionIdAcrossMultipleRegistrations)
{
  SessionState s;

  s.setCurrentSessionId(1);
  s.registerNode(1);
  s.registerNode(2);

  s.setCurrentSessionId(2);
  s.registerNode(3);

  EXPECT_EQ(s.getSessionId(1), 1);
  EXPECT_EQ(s.getSessionId(2), 1);
  EXPECT_EQ(s.getSessionId(3), 2);
}

TEST(SessionStateTest, GetAllNodeSessionIdsReturnsAllRegistered)
{
  SessionState s;
  s.setCurrentSessionId(5);
  s.registerNode(10);
  s.registerNode(20);

  const auto& all = s.getAllNodeSessionIds();
  EXPECT_EQ(all.size(), 2u);
  EXPECT_EQ(all.at(10), 5);
  EXPECT_EQ(all.at(20), 5);
}

TEST(SessionStateTest, SetAllReplacesBothMaps)
{
  SessionState s;
  s.setCurrentSessionId(1);
  s.registerNode(1);

  SessionState::NodeSessionMap new_nodes{{100, 7}};
  SessionState::SessionPolygonMap new_polys;
  s.setAll(new_nodes, new_polys);

  EXPECT_EQ(s.getSessionId(1), 0);   // dropped
  EXPECT_EQ(s.getSessionId(100), 7); // new
}

// ---- SMapper wiring ----

TEST(SMapperSessionStateTest, AccessorReturnsUsableState)
{
  mapper_utils::SMapper smapper;
  smapper.sessionState().setCurrentSessionId(4);
  smapper.sessionState().registerNode(42);

  EXPECT_EQ(smapper.sessionState().getSessionId(42), 4);
  EXPECT_EQ(smapper.sessionState().getSessionId(99), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
