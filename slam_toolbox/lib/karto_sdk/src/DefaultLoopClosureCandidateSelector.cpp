/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <karto_sdk/DefaultLoopClosureCandidateSelector.h>
#include <karto_sdk/Mapper.h>

namespace karto
{

  DefaultLoopClosureCandidateSelector::DefaultLoopClosureCandidateSelector(const Mapper* pMapper)
    : m_pMapper(pMapper)
  {
  }

  LocalizedRangeScanVector DefaultLoopClosureCandidateSelector::FindCandidates(
    LocalizedRangeScan* pScan,
    const LocalizedRangeScanMap& rAllScans,
    const LocalizedRangeScanVector& rNearLinkedScans,
    kt_int32u& rStartNum)
  {
    kt_double maxDistance = m_pMapper->getParamLoopSearchMaximumDistance();
    kt_int32u minChainSize = static_cast<kt_int32u>(m_pMapper->getParamLoopMatchMinimumChainSize());
    kt_bool useScanBarycenter = m_pMapper->getParamUseScanBarycenter();

    LocalizedRangeScanVector chain;
    Pose2 pose = pScan->GetReferencePose(useScanBarycenter);

    kt_int32u nScans = static_cast<kt_int32u>(rAllScans.size());
    for (; rStartNum < nScans; rStartNum++)
    {
      auto it = rAllScans.find(static_cast<int>(rStartNum));
      if (it == rAllScans.end() || it->second == nullptr)
      {
        continue;
      }
      LocalizedRangeScan* pCandidateScan = it->second;

      // Skip candidates that no longer own their position (superseded by a
      // later remapping session).
      if (shouldSkipCandidate(pCandidateScan))
      {
        chain.clear();
        continue;
      }

      Pose2 candidateScanPose = pCandidateScan->GetReferencePose(useScanBarycenter);
      kt_double squaredDistance =
        candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());

      if (squaredDistance < math::Square(maxDistance) + KT_TOLERANCE)
      {
        if (find(rNearLinkedScans.begin(), rNearLinkedScans.end(),
                 pCandidateScan) != rNearLinkedScans.end())
        {
          chain.clear();
        }
        else
        {
          chain.push_back(pCandidateScan);
        }
      }
      else
      {
        if (chain.size() >= minChainSize)
        {
          return chain;
        }
        else
        {
          chain.clear();
        }
      }
    }
    return chain;
  }

}  // namespace karto
