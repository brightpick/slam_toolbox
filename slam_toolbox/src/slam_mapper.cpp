/*
 * slam_mapper
 * Copyright (c) 2018, Simbe Robotics
 * Copyright (c) 2018, Steve Macenski
 * Copyright (c) 2019, Samsung Research America
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

#include "slam_toolbox/slam_mapper.hpp"
#include "slam_toolbox/polygon_fill.hpp"
#include <algorithm>

namespace mapper_utils
{

/*****************************************************************************/
SMapper::SMapper()
/*****************************************************************************/
{
  mapper_ = std::make_unique<karto::Mapper>();
}

/*****************************************************************************/
SMapper::~SMapper()
/*****************************************************************************/
{
  mapper_.reset();
}

/*****************************************************************************/
karto::Mapper* SMapper::getMapper()
/*****************************************************************************/
{
  return mapper_.get();
}

/*****************************************************************************/
void SMapper::setMapper(karto::Mapper* mapper)
/*****************************************************************************/
{
  mapper_.reset(mapper);
}

/*****************************************************************************/
void SMapper::clearLocalizationBuffer()
/*****************************************************************************/
{
  mapper_->ClearLocalizationBuffer();
}

/*****************************************************************************/
karto::OccupancyGrid* SMapper::getOccupancyGrid(const double& resolution)
/*****************************************************************************/
{
  const karto::LocalizedRangeScanVector& scans = mapper_->GetAllProcessedScans();

  if (!remapping_)
  {
    return karto::OccupancyGrid::CreateFromScans(scans, resolution);
  }

  // Compute grid dimensions from BASE-SESSION scans only — scans that belong
  // to the current remapping session are excluded.  This keeps the occupancy
  // grid's footprint and origin locked to the previously-saved map, so the
  // remap output slots into the old PGM pixel-for-pixel.  Growing the grid
  // would shift its origin (sub-pixel), which ripples into Bresenham
  // differences on every cell and makes the diff outside the polygon look
  // noisy even though the filter is working.
  karto::LocalizedRangeScanVector base_scans;
  base_scans.reserve(scans.size());
  for (auto* s : scans)
  {
    const slam_toolbox::SessionLabel* label = getLabel(s->GetUniqueId());
    const int sid = label ? label->session_id : 0;
    if (sid != remapping_->current_session_id)
    {
      base_scans.push_back(s);
    }
  }
  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  karto::OccupancyGrid::ComputeDimensions(base_scans, resolution, width, height, offset);
  buildOwnershipImage(width, height, offset, resolution);

  // Construct the grid directly with the base-scan bounds, then render ALL
  // scans through the ownership filter.  Do NOT use the static
  // CreateFromScans(scans,...) — that path internally calls ComputeDimensions
  // on the full scan vector, which would re-grow the footprint and shift the
  // origin sub-pixel.
  auto* result = new karto::OccupancyGrid(width, height, offset, resolution);
  result->CreateFromScans(
    scans,
    ownership_image_.grid(),
    [this](karto::LocalizedRangeScan* pScan) -> kt_int32s
    {
      const slam_toolbox::SessionLabel* label = getLabel(pScan->GetUniqueId());
      return label ? label->session_id : 0;
    });
  return result;
}

/*****************************************************************************/
int SMapper::computeNextSessionId() const
/*****************************************************************************/
{
  int max_sid = 0;
  for (const auto& [node_id, label] : node_labels_)
  {
    max_sid = std::max(max_sid, label.session_id);
  }
  return max_sid + 1;
}

/*****************************************************************************/
bool SMapper::setRemapping(std::vector<karto::Vector2<kt_double>> polygon)
/*****************************************************************************/
{
  if (!slam_toolbox::polygon_fill::isSimplePolygon(polygon))
  {
    ROS_ERROR("SMapper::setRemapping: rejected polygon with %zu vertices — "
              "it must have at least 3 vertices and must not self-intersect.",
              polygon.size());
    return false;
  }

  RemappingConfig cfg;
  cfg.current_session_id = computeNextSessionId();
  cfg.current_polygon = std::move(polygon);
  remapping_ = cfg;

  // Tag new scans with the computed session_id and carry the polygon on the
  // label so it is serialized to .labels on save.
  current_session_label_.session_id = cfg.current_session_id;
  current_session_label_.polygon = cfg.current_polygon;
  return true;
}

/*****************************************************************************/
const std::optional<SMapper::RemappingConfig>& SMapper::getRemapping() const
/*****************************************************************************/
{
  return remapping_;
}

/*****************************************************************************/
bool SMapper::isRemappingNode(int unique_id) const
/*****************************************************************************/
{
  if (!remapping_) return false;
  const slam_toolbox::SessionLabel* label = getLabel(unique_id);
  const int session_id = label ? label->session_id : 0;
  return session_id == remapping_->current_session_id;
}

/*****************************************************************************/
int SMapper::getOwnerAtWorldPosition(const karto::Vector2<kt_double>& position) const
/*****************************************************************************/
{
  return ownership_image_.ownerAtWorld(position);
}

/*****************************************************************************/
void SMapper::buildOwnershipImage(kt_int32s width, kt_int32s height,
                                   const karto::Vector2<kt_double>& offset,
                                   kt_double resolution)
/*****************************************************************************/
{
  if (!remapping_) return;
  ownership_image_.build(width, height, offset, resolution,
                         node_labels_,
                         remapping_->current_session_id,
                         remapping_->current_polygon);
}

/*****************************************************************************/
tf2::Transform SMapper::toTfPose(const karto::Pose2& pose) const
/*****************************************************************************/
{
  tf2::Transform new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., pose.GetHeading());
  new_pose.setRotation(q);
  return new_pose;
};

/*****************************************************************************/
karto::Pose2 SMapper::toKartoPose(const tf2::Transform& pose) const
/*****************************************************************************/
{
  karto::Pose2 transformed_pose;
  transformed_pose.SetX(pose.getOrigin().x());
  transformed_pose.SetY(pose.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
  return transformed_pose;
};

/*****************************************************************************/
void SMapper::configure(const ros::NodeHandle& nh)
/*****************************************************************************/
{
  bool use_scan_matching;
  if(nh.getParam("use_scan_matching", use_scan_matching))
  {
    mapper_->setParamUseScanMatching(use_scan_matching);
  }

  bool use_scan_barycenter;
  if(nh.getParam("use_scan_barycenter", use_scan_barycenter))
  {
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);
  }

  double minimum_travel_distance = 0.5;
  if(nh.getParam("minimum_travel_distance", minimum_travel_distance))
  {
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);
  }

  double minimum_travel_heading;
  if(nh.getParam("minimum_travel_heading", minimum_travel_heading))
  {
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);
  }

  int scan_buffer_size;
  if(nh.getParam("scan_buffer_size", scan_buffer_size))
  {
    mapper_->setParamScanBufferSize(scan_buffer_size);
  }

  double scan_buffer_maximum_scan_distance;
  if(nh.getParam("scan_buffer_maximum_scan_distance",
    scan_buffer_maximum_scan_distance))
  {
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);
  }

  double link_match_minimum_response_fine;
  if(nh.getParam("link_match_minimum_response_fine",
    link_match_minimum_response_fine))
  {
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);
  }

  double link_scan_maximum_distance;
  if(nh.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
  {
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);
  }

  double loop_search_maximum_distance;
  if(nh.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
  {
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);
  }

  bool do_loop_closing;
  if(nh.getParam("do_loop_closing", do_loop_closing))
  {
    mapper_->setParamDoLoopClosing(do_loop_closing);
  }

  int loop_match_minimum_chain_size;
  if(nh.getParam("loop_match_minimum_chain_size",
    loop_match_minimum_chain_size))
  {
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);
  }

  double loop_match_maximum_variance_coarse;
  if(nh.getParam("loop_match_maximum_variance_coarse",
    loop_match_maximum_variance_coarse))
  {
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);
  }

  double loop_match_minimum_response_coarse;
  if(nh.getParam("loop_match_minimum_response_coarse",
    loop_match_minimum_response_coarse))
  {
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);
  }

  double loop_match_minimum_response_fine;
  if(nh.getParam("loop_match_minimum_response_fine",
    loop_match_minimum_response_fine))
  {
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);
  }

  // Setting Correlation Parameters
  double correlation_search_space_dimension;
  if(nh.getParam("correlation_search_space_dimension",
    correlation_search_space_dimension))
  {
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);
  }

  double correlation_search_space_resolution;
  if(nh.getParam("correlation_search_space_resolution",
    correlation_search_space_resolution))
  {
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);
  }

  double correlation_search_space_smear_deviation;
  if(nh.getParam("correlation_search_space_smear_deviation",
    correlation_search_space_smear_deviation))
  {
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(
      correlation_search_space_smear_deviation);
  }

  // Setting Correlation Parameters, Loop Closure Parameters
  double loop_search_space_dimension;
  if(nh.getParam("loop_search_space_dimension", loop_search_space_dimension))
  {
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);
  }

  double loop_search_space_resolution;
  if(nh.getParam("loop_search_space_resolution", loop_search_space_resolution))
  {
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);
  }

  double loop_search_space_smear_deviation;
  if(nh.getParam("loop_search_space_smear_deviation",
    loop_search_space_smear_deviation))
  {
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);
  }

  // Setting Scan Matcher Parameters
  double distance_variance_penalty;
  if(nh.getParam("distance_variance_penalty", distance_variance_penalty))
  {
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);
  }

  double angle_variance_penalty;
  if(nh.getParam("angle_variance_penalty", angle_variance_penalty))
  {
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);
  }

  double fine_search_angle_offset;
  if(nh.getParam("fine_search_angle_offset", fine_search_angle_offset))
  {
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);
  }

  double coarse_search_angle_offset;
  if(nh.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
  {
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);
  }

  double coarse_angle_resolution;
  if(nh.getParam("coarse_angle_resolution", coarse_angle_resolution))
  {
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);
  }

  double minimum_angle_penalty;
  if(nh.getParam("minimum_angle_penalty", minimum_angle_penalty))
  {
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);
  }

  double minimum_distance_penalty;
  if(nh.getParam("minimum_distance_penalty", minimum_distance_penalty))
  {
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);
  }

  bool use_response_expansion;
  if(nh.getParam("use_response_expansion", use_response_expansion))
  {
    mapper_->setParamUseResponseExpansion(use_response_expansion);
  }
  return;
}

/*****************************************************************************/
void SMapper::setCandidateSelector(karto::LoopClosureCandidateSelector* selector)
/*****************************************************************************/
{
  mapper_->SetCandidateSelector(selector);
}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  mapper_->Reset();
  return;
}

void SMapper::setSessionLabel(const slam_toolbox::SessionLabel& label)
{
  current_session_label_ = label;
}

void SMapper::registerNode(int unique_id)
{
  node_labels_[unique_id] = current_session_label_;
}

const slam_toolbox::SessionLabel* SMapper::getLabel(int unique_id) const
{
  auto it = node_labels_.find(unique_id);
  if (it == node_labels_.end())
  {
    return nullptr;
  }
  return &it->second;
}

const std::unordered_map<int, slam_toolbox::SessionLabel>& SMapper::getAllLabels() const
{
  return node_labels_;
}

void SMapper::setAllLabels(
  const std::unordered_map<int, slam_toolbox::SessionLabel>& labels)
{
  node_labels_ = labels;

  // If remapping was configured before labels were loaded (typical startup
  // path: setParams → deserialize), recompute current_session_id now that the
  // real label history is visible.  Loaded labels may include session_ids
  // larger than whatever we computed against an empty map.
  if (remapping_)
  {
    remapping_->current_session_id = computeNextSessionId();
    current_session_label_.session_id = remapping_->current_session_id;
  }
}


} // end namespace
