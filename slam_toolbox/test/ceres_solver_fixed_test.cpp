/*
 * Copyright (c) 2024, Brightpick
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <karto_sdk/Karto.h>
#include <karto_sdk/Mapper.h>
#include "slam_toolbox/session_label.hpp"
#include "slam_toolbox/slam_mapper.hpp"
#include "../solvers/ceres_solver.hpp"

namespace
{

// Build a minimal LocalizedRangeScan at (x, y, 0) with the given unique ID.
// The range readings are dummies — CeresSolver only reads GetCorrectedPose()
// and GetUniqueId(), so actual scan data does not matter here.
karto::LocalizedRangeScan* makeScan(int id, double x, double y)
{
  karto::RangeReadingsVector readings(10, 1.0);
  auto* scan = new karto::LocalizedRangeScan(karto::Name("test_laser"), readings);
  scan->SetUniqueId(id);
  scan->SetCorrectedPose(karto::Pose2(x, y, 0.0));
  return scan;
}

// Build a constraint edge between two vertices.
// LinkInfo(pose1, pose2, cov) internally computes GetPoseDifference() as the
// transform of pose2 into the frame of pose1.  For heading=0 this is simply
// (pose2 - pose1).
karto::Edge<karto::LocalizedRangeScan>* makeEdge(
    karto::Vertex<karto::LocalizedRangeScan>* src,
    karto::Vertex<karto::LocalizedRangeScan>* tgt,
    karto::Pose2 pose1,
    karto::Pose2 pose2)
{
  karto::Matrix3 cov;
  cov(0, 0) = 1e-3;
  cov(1, 1) = 1e-3;
  cov(2, 2) = 1e-3;
  auto* edge = new karto::Edge<karto::LocalizedRangeScan>(src, tgt);
  edge->SetLabel(new karto::LinkInfo(pose1, pose2, cov));
  return edge;
}

} // namespace

// ---------------------------------------------------------------------------
// Test scenario
// ---------------------------------------------------------------------------
//
//  Session IDs:
//    Node 0  (0, 0)  session_id=0 — fixed by CeresSolver first_node_ mechanism
//                                   AND pinned by non_fixed_session_ids (not in list)
//    Node 1  (1, 0)  session_id=0 — pinned (session_id not in non_fixed list)
//    Node 2  (3, 0)  session_id=1 — free (session_id=1 IS in non_fixed list)
//
//  non_fixed_session_ids = {1}  →  only session 1 is allowed to move
//
//  Constraints:
//    Edge 0→1 : diff (1, 0)  — consistent with initial positions
//    Edge 1→2 : diff (2, 0)  — consistent with initial positions
//    Edge 0→2 : diff (2, 0)  — loop closure, INCONSISTENT
//                               (says node 2 should be at (2,0) from node 0,
//                                but node 2 is actually at (3,0))
//
//  Expected after Compute():
//    Node 0: (0, 0)  — unchanged (first_node_ fix + session pinned)
//    Node 1: (1, 0)  — unchanged (session_id=0 not in non_fixed list)
//    Node 2: moves toward (2, 0) — session_id=1 is free; loop closure pulls it
// ---------------------------------------------------------------------------

TEST(CeresSolverFixedPoseTest, SessionBasedPinningPreventsMovement)
{
  // --- Build scans and vertices ---
  auto* scan0 = makeScan(0, 0.0, 0.0);
  auto* scan1 = makeScan(1, 1.0, 0.0);
  auto* scan2 = makeScan(2, 3.0, 0.0);

  auto* v0 = new karto::Vertex<karto::LocalizedRangeScan>(scan0);
  auto* v1 = new karto::Vertex<karto::LocalizedRangeScan>(scan1);
  auto* v2 = new karto::Vertex<karto::LocalizedRangeScan>(scan2);

  // --- Build constraints ---
  auto* e01 = makeEdge(v0, v1, karto::Pose2(0.0, 0.0, 0.0), karto::Pose2(1.0, 0.0, 0.0));
  auto* e12 = makeEdge(v1, v2, karto::Pose2(1.0, 0.0, 0.0), karto::Pose2(3.0, 0.0, 0.0));
  auto* e02 = makeEdge(v0, v2, karto::Pose2(0.0, 0.0, 0.0), karto::Pose2(2.0, 0.0, 0.0));

  // --- Set up SMapper: session 0 = pinned, session 1 = free ---
  mapper_utils::SMapper smapper;

  slam_toolbox::SessionLabel session0_label;
  session0_label.session_id = 0;
  smapper.setSessionLabel(session0_label);
  smapper.registerNode(0);
  smapper.registerNode(1);

  slam_toolbox::SessionLabel session1_label;
  session1_label.session_id = 1;
  smapper.setSessionLabel(session1_label);
  smapper.registerNode(2);

  // Configure remapping: session 1 is free to move; bbox is arbitrary (ceres only
  // uses non_fixed_session_ids, not the spatial boundary).
  {
    mapper_utils::SMapper::RemappingConfig cfg;
    cfg.non_fixed_session_ids = {1};
    cfg.bbox.SetMinimum(karto::Vector2<kt_double>(0.0, 0.0));
    cfg.bbox.SetMaximum(karto::Vector2<kt_double>(100.0, 100.0));
    smapper.setRemapping(std::move(cfg));
  }

  // --- Set up solver ---
  solver_plugins::CeresSolver solver;
  solver.setMapper(&smapper);

  // AddNode order: node 0 first so it becomes first_node_
  solver.AddNode(v0);
  solver.AddNode(v1);
  solver.AddNode(v2);

  solver.AddConstraint(e01);
  solver.AddConstraint(e12);
  solver.AddConstraint(e02);

  // --- Run optimisation ---
  solver.Compute();

  // --- Verify ---
  auto* graph = solver.getGraph();
  ASSERT_NE(graph, nullptr);

  const double tol = 1e-6;

  // Node 0: must stay at origin (first_node_ mechanism + session 0 pinned)
  ASSERT_NE(graph->find(0), graph->end());
  EXPECT_NEAR((*graph)[0](0), 0.0, tol);
  EXPECT_NEAR((*graph)[0](1), 0.0, tol);

  // Node 1: must not have moved (session_id=0, not in non_fixed_session_ids)
  ASSERT_NE(graph->find(1), graph->end());
  EXPECT_NEAR((*graph)[1](0), 1.0, tol);
  EXPECT_NEAR((*graph)[1](1), 0.0, tol);

  // Node 2: must have moved — session_id=1 is free; loop closure pulls it away from (3,0)
  ASSERT_NE(graph->find(2), graph->end());
  EXPECT_GT(std::abs((*graph)[2](0) - 3.0), tol);

  // --- Cleanup ---
  delete e01;
  delete e12;
  delete e02;
  delete v0;
  delete v1;
  delete v2;
  delete scan0;
  delete scan1;
  delete scan2;
}

// ---------------------------------------------------------------------------
// When non_fixed_session_ids is empty, no extra pinning — only first node fixed.
// ---------------------------------------------------------------------------
//
//  All nodes have session_id=0, non_fixed_session_ids = {} (empty).
//  Only node 0 is pinned (first_node_). Nodes 1 and 2 are free to move.
//
//  Constraints:
//    Edge 0→1 : diff (1, 0)
//    Edge 0→2 : diff (2, 0)  — loop closure: says node 2 is at (2,0) from node 0,
//                               but initial position is (5,0) → should move
// ---------------------------------------------------------------------------

TEST(CeresSolverFixedPoseTest, EmptyNonFixedListOnlyPinsFirstNode)
{
  auto* scan0 = makeScan(0, 0.0, 0.0);
  auto* scan1 = makeScan(1, 1.0, 0.0);
  auto* scan2 = makeScan(2, 5.0, 0.0);

  auto* v0 = new karto::Vertex<karto::LocalizedRangeScan>(scan0);
  auto* v1 = new karto::Vertex<karto::LocalizedRangeScan>(scan1);
  auto* v2 = new karto::Vertex<karto::LocalizedRangeScan>(scan2);

  auto* e01 = makeEdge(v0, v1, karto::Pose2(0.0, 0.0, 0.0), karto::Pose2(1.0, 0.0, 0.0));
  auto* e02 = makeEdge(v0, v2, karto::Pose2(0.0, 0.0, 0.0), karto::Pose2(2.0, 0.0, 0.0));

  mapper_utils::SMapper smapper;
  slam_toolbox::SessionLabel label;
  label.session_id = 0;
  smapper.setSessionLabel(label);
  smapper.registerNode(0);
  smapper.registerNode(1);
  smapper.registerNode(2);
  // non_fixed_session_ids left empty (default)

  solver_plugins::CeresSolver solver;
  solver.setMapper(&smapper);

  solver.AddNode(v0);
  solver.AddNode(v1);
  solver.AddNode(v2);
  solver.AddConstraint(e01);
  solver.AddConstraint(e02);

  solver.Compute();

  auto* graph = solver.getGraph();
  ASSERT_NE(graph, nullptr);

  const double tol = 1e-6;

  // Node 0: pinned as first node
  ASSERT_NE(graph->find(0), graph->end());
  EXPECT_NEAR((*graph)[0](0), 0.0, tol);
  EXPECT_NEAR((*graph)[0](1), 0.0, tol);

  // Node 2: must have moved toward (2,0) — no extra pinning in effect
  ASSERT_NE(graph->find(2), graph->end());
  EXPECT_GT(std::abs((*graph)[2](0) - 5.0), tol);

  delete e01;
  delete e02;
  delete v0;
  delete v1;
  delete v2;
  delete scan0;
  delete scan1;
  delete scan2;
}

int main(int argc, char** argv)
{
  // CeresSolver constructor uses ros::NodeHandle to read solver parameters.
  ros::init(argc, argv, "ceres_solver_fixed_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
