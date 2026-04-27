/*
 * Copyright (c) 2026, Brightpick
 *
 * YAML read/write for the sidecar `.labels` file that pairs with a saved
 * posegraph.  Layout:
 *
 *   sessions:
 *     - id: <int>
 *       polygon: [[x1, y1], [x2, y2], ...]
 *     ...
 *   labels:
 *     - id: <pose_id>
 *       session_id: <int>
 *     ...
 *
 * Sessions and labels are stored separately: polygons are emitted once per
 * session, and labels just record which session each pose belongs to.
 * kBaseSessionId is the implicit default — it never appears in `sessions`,
 * and its labels are omitted from `labels`.  Any pose id absent from the
 * file is treated as kBaseSessionId.
 */

#ifndef SLAM_TOOLBOX_LABELS_SERIALIZATION_H_
#define SLAM_TOOLBOX_LABELS_SERIALIZATION_H_

#include <fstream>
#include <map>
#include <string>
#include <sys/stat.h>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "slam_toolbox/remapping/polygon_fill.hpp"
#include "slam_toolbox/remapping/remapping_state.hpp"

namespace slam_toolbox
{

inline void saveLabels(const std::string& filename,
  const NodeSessionMap& node_session_ids,
  const SessionPolygonMap& session_polygons)
{
  YAML::Node root;

  // Emit sessions in session_id order (std::map) for determinism.
  std::map<int, const Polygon*> ordered;
  for (const auto& [sid, polygon] : session_polygons)
  {
    ordered[sid] = &polygon;
  }
  for (const auto& [sid, polyPtr] : ordered)
  {
    YAML::Node session;
    session["id"] = sid;
    YAML::Node poly;
    for (const auto& v : *polyPtr)
    {
      YAML::Node vertex;
      vertex.push_back(v.GetX());
      vertex.push_back(v.GetY());
      vertex.SetStyle(YAML::EmitterStyle::Flow);
      poly.push_back(vertex);
    }
    session["polygon"] = poly;
    root["sessions"].push_back(session);
  }

  for (const auto& [pose_id, session_id] : node_session_ids)
  {
    if (session_id == kBaseSessionId) continue;  // base session is implicit
    YAML::Node entry;
    entry["id"] = pose_id;
    entry["session_id"] = session_id;
    root["labels"].push_back(entry);
  }

  std::ofstream fout(filename);
  fout << root;
}

inline bool loadLabels(const std::string& filename,
  NodeSessionMap& node_session_ids,
  SessionPolygonMap& session_polygons)
{
  struct stat buffer;
  if (stat(filename.c_str(), &buffer) != 0)
  {
    return false;
  }
  try
  {
    YAML::Node root = YAML::LoadFile(filename);

    // Guard against the top-level keys being present-but-not-a-sequence
    // (e.g. an empty `sessions:` parses as a null scalar).  Iterating a
    // non-sequence throws YAML::InvalidNode, which would drop the whole
    // otherwise-valid file via the outer catch.
    const YAML::Node sessions_node = root["sessions"];
    if (sessions_node.IsDefined() && sessions_node.IsSequence())
    {
      for (const auto& session : sessions_node)
      {
        const int sid = session["id"].as<int>();

        const auto& poly = session["polygon"];
        if (!poly || !poly.IsSequence())
        {
          ROS_ERROR("loadLabels: session %d has no polygon — skipped.", sid);
          continue;
        }

        Polygon polygon;
        bool malformed = false;
        for (const auto& vertex : poly)
        {
          if (!vertex.IsSequence() || vertex.size() != 2)
          {
            ROS_ERROR("loadLabels: session %d has a malformed vertex "
                      "— session skipped.", sid);
            malformed = true;
            break;
          }
          polygon.emplace_back(vertex[0].as<double>(), vertex[1].as<double>());
        }
        if (malformed) continue;

        if (polygon.size() < 3)
        {
          ROS_ERROR("loadLabels: session %d polygon has %zu "
                    "vertex(es) — at least 3 required, session skipped.",
                    sid, polygon.size());
          continue;
        }

        if (!isSimplePolygon(polygon))
        {
          ROS_ERROR("loadLabels: session %d polygon is "
                    "self-intersecting — session skipped.", sid);
          continue;
        }

        session_polygons[sid] = std::move(polygon);
      }
    }

    const YAML::Node labels_node = root["labels"];
    if (labels_node.IsDefined() && labels_node.IsSequence())
    {
      for (const auto& entry : labels_node)
      {
        const int pose_id = entry["id"].as<int>();
        const int session_id = entry["session_id"].as<int>();
        node_session_ids[pose_id] = session_id;
      }
    }
  }
  catch (const YAML::Exception& e)
  {
    ROS_WARN("loadLabels: failed to read labels file: %s. "
             "Continuing without labels.", e.what());
    return false;
  }
  return true;
}

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_LABELS_SERIALIZATION_H_
