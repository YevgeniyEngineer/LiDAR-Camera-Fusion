#pragma once

#include "lidar_return.hpp"

#include <common_library/containers/static_vector.hpp>

#include <cstdint>

namespace processing
{
class Pipeline
{
  public:
    /// Theoretical maximum number of points in the point cloud frame
    static constexpr std::size_t MAX_POINTS = 500'000;

    Pipeline() = default;
    ~Pipeline() = default;

  private:
    common_library::containers::StaticVector<LidarReturn, MAX_POINTS> input_point_cloud_;
};
} // namespace processing