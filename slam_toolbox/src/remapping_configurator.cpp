/*
 * Copyright (c) 2026, Brightpick
 */

#include "slam_toolbox/remapping_configurator.hpp"

#include <XmlRpcValue.h>

namespace slam_toolbox
{

namespace
{

// XmlRpcValue's double cast only accepts TypeDouble — integer YAML scalars
// (e.g. 916 vs 916.0) come through as TypeInt and would otherwise throw.
std::optional<double> toDouble(XmlRpc::XmlRpcValue& v)
{
  switch (v.getType())
  {
    case XmlRpc::XmlRpcValue::TypeDouble: return static_cast<double>(v);
    case XmlRpc::XmlRpcValue::TypeInt:    return static_cast<int>(v);
    default: return std::nullopt;
  }
}

bool readPolygonParam(XmlRpc::XmlRpcValue& xml,
                      std::vector<karto::Vector2<kt_double>>& out)
{
  if (xml.getType() != XmlRpc::XmlRpcValue::TypeArray || xml.size() < 3)
  {
    ROS_ERROR("RemappingConfigurator: remapping_polygon must be a list of "
              "at least 3 [x, y] vertices.");
    return false;
  }
  out.clear();
  out.reserve(xml.size());
  for (int i = 0; i < xml.size(); ++i)
  {
    if (xml[i].getType() != XmlRpc::XmlRpcValue::TypeArray || xml[i].size() != 2)
    {
      ROS_ERROR("RemappingConfigurator: remapping_polygon vertex %d must be [x, y].", i);
      return false;
    }
    const auto x = toDouble(xml[i][0]);
    const auto y = toDouble(xml[i][1]);
    if (!x || !y)
    {
      ROS_ERROR("RemappingConfigurator: remapping_polygon vertex %d has "
                "non-numeric coordinate.", i);
      return false;
    }
    out.emplace_back(*x, *y);
  }
  return true;
}

}  // namespace

void RemappingConfigurator::loadFromRosParams(
  ros::NodeHandle& nh,
  mapper_utils::SMapper& smapper,
  karto::LoopClosureCandidateSelector* selector)
{
  XmlRpc::XmlRpcValue xml_poly;
  if (!nh.getParam("remapping_polygon", xml_poly)) return;

  std::vector<karto::Vector2<kt_double>> polygon;
  if (!readPolygonParam(xml_poly, polygon)) return;

  std::string units;
  nh.param<std::string>("remapping_polygon_units", units, "pixels");

  if (units == "world")
  {
    if (smapper.sessionState().setRemapping(polygon))
    {
      ROS_INFO("RemappingConfigurator: remapping configured (world) — "
               "session_id=%d (auto), %zu-vertex polygon.",
               smapper.sessionState().getRemapping()->current_session_id, polygon.size());
      installLoopClosureFilter(selector, smapper);
    }
  }
  else if (units == "pixels")
  {
    pending_pixel_polygon_ = std::move(polygon);
    ROS_INFO("RemappingConfigurator: remapping_polygon queued in pixels "
             "(%zu vertices) — will resolve to world coords after "
             "deserialization.", pending_pixel_polygon_->size());
  }
  else
  {
    ROS_ERROR("RemappingConfigurator: remapping_polygon_units must be "
              "'pixels' or 'world' (got '%s').  Remapping disabled.",
              units.c_str());
  }
}

void RemappingConfigurator::resolvePendingPolygon(
  mapper_utils::SMapper& smapper,
  double resolution,
  karto::LoopClosureCandidateSelector* selector)
{
  if (!pending_pixel_polygon_) return;

  // PGM convention: py=0 is the top row, y increasing downward; the grid's
  // row 0 is the bottom, so image_row → grid_row is (height - 1 - py).
  // Per vertex:
  //   world_x = offset.x + px * resolution
  //   world_y = offset.y + (height - 1 - py) * resolution
  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  karto::OccupancyGrid::ComputeDimensions(
    smapper.getMapper()->GetAllProcessedScans(),
    resolution, width, height, offset);

  if (height <= 0 || width <= 0)
  {
    ROS_WARN("RemappingConfigurator: pending pixel remapping_polygon "
             "cannot be applied — deserialized map has zero dimensions.");
  }
  else
  {
    std::vector<karto::Vector2<kt_double>> world_poly;
    world_poly.reserve(pending_pixel_polygon_->size());
    for (const auto& v : *pending_pixel_polygon_)
    {
      const double wx = offset.GetX() + v.GetX() * resolution;
      const double wy = offset.GetY() + (height - 1 - v.GetY()) * resolution;
      world_poly.emplace_back(wx, wy);
    }
    if (smapper.sessionState().setRemapping(world_poly))
    {
      ROS_INFO("RemappingConfigurator: remapping resolved from pixels — "
               "session_id=%d (auto), %zu-vertex polygon (grid %dx%d, "
               "offset [%.3f, %.3f], resolution %.3f)",
               smapper.sessionState().getRemapping()->current_session_id,
               world_poly.size(),
               width, height, offset.GetX(), offset.GetY(), resolution);
      installLoopClosureFilter(selector, smapper);
    }
  }
  pending_pixel_polygon_.reset();
}

void RemappingConfigurator::installFixedPosePredicate(
  karto::ScanSolver& solver,
  mapper_utils::SMapper& smapper)
{
  solver.setNodeFixedPredicate(
    [&smapper](int id) {
      return smapper.sessionState().getRemapping().has_value() && !smapper.sessionState().isRemappingNode(id);
    });
}

void RemappingConfigurator::installLoopClosureFilter(
  karto::LoopClosureCandidateSelector* selector,
  mapper_utils::SMapper& smapper)
{
  if (!selector) return;

  selector->setCandidateFilter(
    [&smapper](karto::LocalizedRangeScan* pScan) -> bool
    {
      const int scan_sid = smapper.sessionState().getSessionId(pScan->GetUniqueId());
      const int owner = smapper.sessionState().ownershipImage().ownerAtWorld(
        pScan->GetCorrectedPose().GetPosition());
      return scan_sid != owner;
    });
}

}  // namespace slam_toolbox
