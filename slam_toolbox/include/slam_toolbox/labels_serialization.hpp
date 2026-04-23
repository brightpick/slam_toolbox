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
 * Polygons are emitted once per session and joined onto each label on load.
 * Session 0 (base) is the implicit default — it never appears in `sessions`,
 * and its labels are omitted from `labels`.  Any pose id absent from the
 * file is treated as session 0 with no polygon; SMapper callers already
 * handle that case by defaulting to session_id = 0.
 */

#ifndef SLAM_TOOLBOX_LABELS_SERIALIZATION_H_
#define SLAM_TOOLBOX_LABELS_SERIALIZATION_H_

#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <karto_sdk/Karto.h>
#include <sys/stat.h>

#include "slam_toolbox/polygon_fill.hpp"
#include "slam_toolbox/session_label.hpp"

namespace slam_toolbox
{
namespace labels_serialization
{

inline bool fileExists(const std::string& name)
{
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

inline void save(const std::string& filename,
  const std::unordered_map<int, SessionLabel>& labels)
{
  // Collect distinct (session_id, polygon) pairs.  std::map keeps the
  // output deterministically ordered by session_id.
  std::map<int, std::vector<karto::Vector2<kt_double>>> sessions;
  for (const auto& [pose_id, label] : labels)
  {
    if (label.polygon.has_value())
    {
      sessions[label.session_id] = *label.polygon;
    }
  }

  YAML::Node root;
  for (const auto& [sid, polygon] : sessions)
  {
    YAML::Node session;
    session["id"] = sid;
    YAML::Node poly;
    for (const auto& v : polygon)
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

  for (const auto& [pose_id, label] : labels)
  {
    if (label.session_id == 0) continue;  // base session is implicit
    YAML::Node entry;
    entry["id"] = pose_id;
    entry["session_id"] = label.session_id;
    root["labels"].push_back(entry);
  }

  std::ofstream fout(filename);
  fout << root;
}

inline bool load(const std::string& filename,
  std::unordered_map<int, SessionLabel>& labels)
{
  if (!fileExists(filename))
  {
    return false;
  }
  try
  {
    YAML::Node root = YAML::LoadFile(filename);

    std::unordered_map<int, std::vector<karto::Vector2<kt_double>>> session_polygons;
    for (const auto& session : root["sessions"])
    {
      const int sid = session["id"].as<int>();

      const auto& poly = session["polygon"];
      if (!poly || !poly.IsSequence())
      {
        ROS_ERROR("labels_serialization: session %d has no polygon — skipped.", sid);
        continue;
      }

      std::vector<karto::Vector2<kt_double>> polygon;
      bool malformed = false;
      for (const auto& vertex : poly)
      {
        if (!vertex.IsSequence() || vertex.size() != 2)
        {
          ROS_ERROR("labels_serialization: session %d has a malformed vertex "
                    "— session skipped.", sid);
          malformed = true;
          break;
        }
        polygon.emplace_back(vertex[0].as<double>(), vertex[1].as<double>());
      }
      if (malformed) continue;

      if (polygon.size() < 3)
      {
        ROS_ERROR("labels_serialization: session %d polygon has %zu "
                  "vertex(es) — at least 3 required, session skipped.",
                  sid, polygon.size());
        continue;
      }

      if (!polygon_fill::isSimplePolygon(polygon))
      {
        ROS_ERROR("labels_serialization: session %d polygon is "
                  "self-intersecting — session skipped.", sid);
        continue;
      }

      session_polygons[sid] = std::move(polygon);
    }

    for (const auto& entry : root["labels"])
    {
      const int pose_id = entry["id"].as<int>();
      SessionLabel label = SessionLabel::deserialize(entry);
      auto it = session_polygons.find(label.session_id);
      if (it != session_polygons.end())
      {
        label.polygon = it->second;
      }
      labels[pose_id] = label;
    }
  }
  catch (const YAML::Exception& e)
  {
    ROS_WARN("labels_serialization: failed to read labels file: %s. "
             "Continuing without labels.", e.what());
    return false;
  }
  return true;
}

}  // namespace labels_serialization
}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX_LABELS_SERIALIZATION_H_
