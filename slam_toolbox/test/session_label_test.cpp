#include <gtest/gtest.h>
#include <unordered_map>
#include "slam_toolbox/session_label.hpp"
#include "slam_toolbox/slam_mapper.hpp"

// ---- SessionLabel ----

TEST(SessionLabelTest, DefaultSessionIdIsZero)
{
  slam_toolbox::SessionLabel label;
  EXPECT_EQ(label.session_id, 0);
}

TEST(SessionLabelTest, SerializeContainsSessionId)
{
  slam_toolbox::SessionLabel label;
  label.session_id = 42;
  YAML::Node node = label.serialize();
  ASSERT_TRUE(node["session_id"]);
  EXPECT_EQ(node["session_id"].as<int>(), 42);
}

TEST(SessionLabelTest, DeserializeReadsSessionId)
{
  YAML::Node node;
  node["session_id"] = 7;
  slam_toolbox::SessionLabel label = slam_toolbox::SessionLabel::deserialize(node);
  EXPECT_EQ(label.session_id, 7);
}

TEST(SessionLabelTest, Roundtrip)
{
  slam_toolbox::SessionLabel original;
  original.session_id = 99;
  slam_toolbox::SessionLabel copy = slam_toolbox::SessionLabel::deserialize(original.serialize());
  EXPECT_EQ(copy.session_id, original.session_id);
}

// ---- SMapper label methods ----

TEST(SMapperLabelTest, GetLabelUnknownNodeReturnsNull)
{
  mapper_utils::SMapper smapper;
  EXPECT_EQ(smapper.getLabel(999), nullptr);
}

TEST(SMapperLabelTest, RegisterNodeAssignsCurrentLabel)
{
  mapper_utils::SMapper smapper;
  slam_toolbox::SessionLabel label;
  label.session_id = 3;
  smapper.setSessionLabel(label);
  smapper.registerNode(10);

  const slam_toolbox::SessionLabel* result = smapper.getLabel(10);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->session_id, 3);
}

TEST(SMapperLabelTest, RegisterMultipleNodesWithDifferentLabels)
{
  mapper_utils::SMapper smapper;

  slam_toolbox::SessionLabel label1;
  label1.session_id = 1;
  smapper.setSessionLabel(label1);
  smapper.registerNode(1);
  smapper.registerNode(2);

  slam_toolbox::SessionLabel label2;
  label2.session_id = 2;
  smapper.setSessionLabel(label2);
  smapper.registerNode(3);

  EXPECT_EQ(smapper.getLabel(1)->session_id, 1);
  EXPECT_EQ(smapper.getLabel(2)->session_id, 1);
  EXPECT_EQ(smapper.getLabel(3)->session_id, 2);
}

TEST(SMapperLabelTest, GetAllLabelsReturnsAllRegistered)
{
  mapper_utils::SMapper smapper;
  slam_toolbox::SessionLabel label;
  label.session_id = 5;
  smapper.setSessionLabel(label);
  smapper.registerNode(10);
  smapper.registerNode(20);

  const auto& all = smapper.getAllLabels();
  EXPECT_EQ(all.size(), 2u);
  EXPECT_EQ(all.at(10).session_id, 5);
  EXPECT_EQ(all.at(20).session_id, 5);
}

TEST(SMapperLabelTest, SetAllLabelsReplacesMap)
{
  mapper_utils::SMapper smapper;
  slam_toolbox::SessionLabel label;
  label.session_id = 1;
  smapper.setSessionLabel(label);
  smapper.registerNode(1);

  std::unordered_map<int, slam_toolbox::SessionLabel> new_labels;
  slam_toolbox::SessionLabel new_label;
  new_label.session_id = 7;
  new_labels[100] = new_label;
  smapper.setAllLabels(new_labels);

  EXPECT_EQ(smapper.getLabel(1), nullptr);
  ASSERT_NE(smapper.getLabel(100), nullptr);
  EXPECT_EQ(smapper.getLabel(100)->session_id, 7);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
