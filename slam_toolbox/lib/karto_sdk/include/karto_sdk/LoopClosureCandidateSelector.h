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

#ifndef karto_sdk_LOOP_CLOSURE_CANDIDATE_SELECTOR_H
#define karto_sdk_LOOP_CLOSURE_CANDIDATE_SELECTOR_H

#include <functional>
#include <karto_sdk/Karto.h>

namespace karto
{
  class Mapper;

  /**
   * Abstract interface for loop closure candidate selection heuristics.
   * Implementations return a chain of candidate scans for one loop closure
   * attempt. TryCloseLoop() calls this iteratively, advancing rStartNum.
   */
  class KARTO_EXPORT LoopClosureCandidateSelector
  {
  public:
    LoopClosureCandidateSelector() {}
    virtual ~LoopClosureCandidateSelector() {}

    /**
     * Find a chain of candidate scans for loop closure.
     * @param pScan             Current scan being processed.
     * @param rAllScans         All scans for this sensor (map<int, LocalizedRangeScan*>).
     * @param rNearLinkedScans  Scans already graph-connected within search distance (pre-computed).
     * @param rStartNum         In/out: index to start from; advanced past returned chain.
     * @return Chain of candidate scans. Empty = no more candidates.
     */
    virtual LocalizedRangeScanVector FindCandidates(
      LocalizedRangeScan* pScan,
      const LocalizedRangeScanMap& rAllScans,
      const LocalizedRangeScanVector& rNearLinkedScans,
      kt_int32u& rStartNum) = 0;

    /**
     * Called when the underlying Mapper object is replaced (e.g. after
     * posegraph deserialization). Implementations that cache a Mapper pointer
     * must override this to update their internal reference; the default
     * no-op is safe for selectors that do not hold a Mapper pointer.
     */
    virtual void setMapper(const Mapper* /*pMapper*/) {}

    /**
     * Set an optional predicate that returns true when a candidate scan
     * should be skipped (e.g. because it no longer owns its grid region
     * after a more recent remapping session claimed the area).
     */
    void setCandidateFilter(std::function<bool(LocalizedRangeScan*)> fn)
    {
      candidate_filter_ = std::move(fn);
    }

  protected:
    /**
     * True when a candidate scan should be skipped according to the
     * filter set via setCandidateFilter().  Safe to call when no filter
     * has been set (returns false).
     */
    bool shouldSkipCandidate(LocalizedRangeScan* pScan) const
    {
      return candidate_filter_ && candidate_filter_(pScan);
    }

  private:
    std::function<bool(LocalizedRangeScan*)> candidate_filter_;
  };

}  // namespace karto

#endif  // karto_sdk_LOOP_CLOSURE_CANDIDATE_SELECTOR_H
