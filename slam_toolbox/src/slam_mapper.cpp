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

#include <algorithm>
#include <iterator>

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
  if (!remapping_state_.getRemappingPolygon())
  {
    return karto::OccupancyGrid::CreateFromScans(scans, resolution);
  }
  return buildRemapGrid(scans, resolution);
}

/*****************************************************************************/
bool SMapper::computeBaseFootprint(
  const karto::LocalizedRangeScanVector& scans, double resolution,
  kt_int32s& width, kt_int32s& height,
  karto::Vector2<kt_double>& offset) const
/*****************************************************************************/
{
  // BASE-session scans only (session 0 — the saved map loaded from disk).
  // Historical remap sessions are deliberately excluded: their scan
  // endpoints can extend the bbox, which would grow the rendered grid
  // across remap cycles and break the "remap output slots into the old PGM
  // pixel-for-pixel" guarantee.  Locking to the base session keeps the
  // footprint and origin identical no matter how many times the user has
  // remapped.
  //
  // Karto's own grid-building path tolerates null entries in the scan
  // vector (see CreateFromScans / ComputeDimensions in Karto.h), so guard
  // with `s &&` here too.
  auto isBaseNode = remapping_state_.makeComputeGridSizePredicate();
  karto::LocalizedRangeScanVector base_scans;
  base_scans.reserve(scans.size());
  std::copy_if(scans.begin(), scans.end(), std::back_inserter(base_scans),
    [&isBaseNode](karto::LocalizedRangeScan* s) {
      return s && isBaseNode(s->GetUniqueId());
    });

  // No base-session scans means we have nothing to anchor the grid
  // footprint to.  In the current flow this can't happen (the
  // start_remapping service rejects when no scans are loaded, and after the
  // first setRemapping every pre-existing scan is a base scan), but
  // ComputeDimensions on an empty vector leaves width/height undefined —
  // guard explicitly.
  if (base_scans.empty()) return false;

  karto::OccupancyGrid::ComputeDimensions(base_scans, resolution, width, height, offset);
  return true;
}

/*****************************************************************************/
void SMapper::rebuildOwnershipImage(double resolution)
/*****************************************************************************/
{
  if (!remapping_state_.getRemappingPolygon()) return;

  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  if (!computeBaseFootprint(
        mapper_->GetAllProcessedScans(), resolution, width, height, offset))
  {
    return;
  }
  remapping_state_.buildOwnershipImage(offset, resolution);
}

/*****************************************************************************/
bool SMapper::getBaseFootprint(double resolution,
                               kt_int32s& width, kt_int32s& height,
                               karto::Vector2<kt_double>& offset) const
/*****************************************************************************/
{
  return computeBaseFootprint(
    mapper_->GetAllProcessedScans(), resolution, width, height, offset);
}

/*****************************************************************************/
karto::OccupancyGrid* SMapper::buildRemapGrid(
  const karto::LocalizedRangeScanVector& scans, double resolution)
/*****************************************************************************/
{
  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  if (!computeBaseFootprint(scans, resolution, width, height, offset)) return nullptr;

  remapping_state_.buildOwnershipImage(offset, resolution);

  // Construct the grid directly with the base-scan bounds, then render ALL
  // scans through the ownership filter.  Do NOT use the static
  // CreateFromScans(scans,...) — that path internally calls ComputeDimensions
  // on the full scan vector, which would re-grow the footprint and shift the
  // origin sub-pixel.
  auto* result = new karto::OccupancyGrid(width, height, offset, resolution);
  result->CreateFromScans(scans, remapping_state_.makeGridCellPredicate());
  return result;
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

  double loop_closure_max_correction;
  if(nh.getParam("loop_closure_max_correction", loop_closure_max_correction))
  {
    mapper_->setParamLoopClosureMaxCorrection(loop_closure_max_correction);
  }

  double loop_closure_max_rotational_correction;
  if(nh.getParam("loop_closure_max_rotational_correction", loop_closure_max_rotational_correction))
  {
    mapper_->setParamLoopClosureMaxRotationalCorrection(loop_closure_max_rotational_correction);
  }

  bool loop_closure_debug_info;
  if(nh.getParam("loop_closure_debug_info", loop_closure_debug_info))
  {
    mapper_->setParamLoopClosureDebugInfo(loop_closure_debug_info);
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

} // end namespace
