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

#ifndef SLAM_TOOLBOX_MAPPER_AWARE_SOLVER_HPP_
#define SLAM_TOOLBOX_MAPPER_AWARE_SOLVER_HPP_

// Forward-declare SMapper to keep this header dependency-free.
// slam_toolbox_common.cpp includes slam_mapper.hpp separately.
namespace mapper_utils { class SMapper; }

namespace slam_toolbox
{

/**
 * Optional mixin for karto::ScanSolver implementations that want to receive
 * the SMapper pointer so they can look up per-node SessionLabels at solve time.
 *
 * toolbox_common casts solver_ to this interface — avoiding a direct dependency
 * on any concrete solver header and the circular link it would create.
 */
class IMapperAwareSolver
{
public:
  virtual void setMapper(const mapper_utils::SMapper* smapper) = 0;
  virtual ~IMapperAwareSolver() = default;
};

} // namespace slam_toolbox

#endif // SLAM_TOOLBOX_MAPPER_AWARE_SOLVER_HPP_
