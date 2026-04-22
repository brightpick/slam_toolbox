/*
 * Author
 * Copyright (c) 2018, Simbe Robotics, Inc.
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

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX_SERIALIZATION_H_
#define SLAM_TOOLBOX_SERIALIZATION_H_

#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <unordered_map>
#include <ros/ros.h>
#include <karto_sdk/Karto.h>
#include <karto_sdk/Mapper.h>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include "slam_toolbox/polygon_fill.hpp"
#include "slam_toolbox/session_label.hpp"

namespace serialization
{

inline bool fileExists(const std::string& name)
{
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

// .labels file layout:
//   sessions:
//     - id: <int>
//       polygon: [[x1, y1], [x2, y2], ...]
//     ...
//   labels:
//     - id: <pose_id>
//       session_id: <int>
//     ...
//
// Polygons are emitted once per session in `sessions` and joined onto each
// label on load.  Session 0 (base) is the implicit default: it never
// appears in `sessions`, and its labels are also omitted from `labels`.
// Any pose id that is absent from the file is treated as session 0 with no
// polygon — the SMapper getLabel/isRemappingNode callers already handle
// the null-label case by defaulting to session_id = 0.

inline void saveLabelsToFile(const std::string& filename,
  const std::unordered_map<int, slam_toolbox::SessionLabel>& labels)
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

inline bool loadLabelsFromFile(const std::string& filename,
  std::unordered_map<int, slam_toolbox::SessionLabel>& labels)
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
        ROS_ERROR("serialization: session %d has no polygon in .labels file "
                  "— skipped.", sid);
        continue;
      }

      std::vector<karto::Vector2<kt_double>> polygon;
      bool malformed = false;
      for (const auto& vertex : poly)
      {
        if (!vertex.IsSequence() || vertex.size() != 2)
        {
          ROS_ERROR("serialization: session %d has a malformed vertex in "
                    ".labels file — session skipped.", sid);
          malformed = true;
          break;
        }
        polygon.emplace_back(vertex[0].as<double>(), vertex[1].as<double>());
      }
      if (malformed) continue;

      if (polygon.size() < 3)
      {
        ROS_ERROR("serialization: session %d polygon has %zu vertex(es) — "
                  "at least 3 required, session skipped.", sid, polygon.size());
        continue;
      }

      if (!slam_toolbox::polygon_fill::isSimplePolygon(polygon))
      {
        ROS_ERROR("serialization: session %d polygon is self-intersecting — "
                  "session skipped.", sid);
        continue;
      }

      session_polygons[sid] = std::move(polygon);
    }

    for (const auto& entry : root["labels"])
    {
      const int pose_id = entry["id"].as<int>();
      slam_toolbox::SessionLabel label = slam_toolbox::SessionLabel::deserialize(entry);
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
    ROS_WARN("serialization: Failed to read labels file: %s. "
      "Continuing without labels.", e.what());
    return false;
  }
  return true;
}

inline void write(const std::string& filename,
  karto::Mapper& mapper,
  karto::Dataset& dataset,
  const std::unordered_map<int, slam_toolbox::SessionLabel>& labels)
{
  try
  {
    mapper.SaveToFile(filename + std::string(".posegraph"));
    dataset.SaveToFile(filename + std::string(".data"));
  }
  catch (boost::archive::archive_exception e)
  {
    ROS_ERROR("Failed to write file: Exception %s", e.what());
  }

  saveLabelsToFile(filename + std::string(".labels"), labels);
}

inline bool read(const std::string& filename,
  karto::Mapper& mapper,
  karto::Dataset& dataset,
  std::unordered_map<int, slam_toolbox::SessionLabel>& labels)
{
  if (!fileExists(filename + std::string(".posegraph")))
  {
    ROS_ERROR("serialization::Read: Failed to open "
      "requested file: %s.", filename.c_str());
    return false;
  }

  try
  {
    mapper.LoadFromFile(filename + std::string(".posegraph"));
    dataset.LoadFromFile(filename + std::string(".data"));
  }
  catch (boost::archive::archive_exception e)
  {
    ROS_ERROR("serialization::Read: Failed to read file: "
      "Exception: %s", e.what());
    return false;
  }

  labels.clear();
  loadLabelsFromFile(filename + std::string(".labels"), labels);

  return true;
}

} // end namespace

#endif //SLAM_TOOLBOX_SERIALIZATION_H_