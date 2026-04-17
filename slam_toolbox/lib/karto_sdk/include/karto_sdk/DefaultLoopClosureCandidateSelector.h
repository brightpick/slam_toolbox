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

#ifndef karto_sdk_DEFAULT_LOOP_CLOSURE_CANDIDATE_SELECTOR_H
#define karto_sdk_DEFAULT_LOOP_CLOSURE_CANDIDATE_SELECTOR_H

#include <karto_sdk/LoopClosureCandidateSelector.h>

namespace karto
{
  class Mapper;

  /** Reimplements the original FindPossibleLoopClosure() spatial-chain heuristic. */
  class KARTO_EXPORT DefaultLoopClosureCandidateSelector
    : public LoopClosureCandidateSelector
  {
  public:
    explicit DefaultLoopClosureCandidateSelector(const Mapper* pMapper);
    virtual ~DefaultLoopClosureCandidateSelector() {}

    LocalizedRangeScanVector FindCandidates(
      LocalizedRangeScan* pScan,
      const LocalizedRangeScanMap& rAllScans,
      const LocalizedRangeScanVector& rNearLinkedScans,
      kt_int32u& rStartNum) override;

    void setMapper(const Mapper* pMapper) override { m_pMapper = pMapper; }

  private:
    const Mapper* m_pMapper;
  };

}  // namespace karto

#endif  // karto_sdk_DEFAULT_LOOP_CLOSURE_CANDIDATE_SELECTOR_H
