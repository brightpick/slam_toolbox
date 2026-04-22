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

#ifndef SLAM_TOOLBOX_SESSION_LABEL_H_
#define SLAM_TOOLBOX_SESSION_LABEL_H_

#include <optional>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <karto_sdk/Karto.h>

namespace slam_toolbox
{

struct SessionLabel
{
  int session_id{0};
  // Area of the session as a simple polygon in world coordinates.
  // nullopt for the base session (no constrained area).
  std::optional<std::vector<karto::Vector2<kt_double>>> polygon;

  // Per-label YAML contains only session_id.  The polygon is emitted once
  // per session in the `sessions:` section of the .labels file — see
  // serialization.hpp.
  YAML::Node serialize() const
  {
    YAML::Node node;
    node["session_id"] = session_id;
    return node;
  }

  static SessionLabel deserialize(const YAML::Node& node)
  {
    SessionLabel label;
    label.session_id = node["session_id"].as<int>();
    return label;
  }
};

} // namespace slam_toolbox

#endif // SLAM_TOOLBOX_SESSION_LABEL_H_
