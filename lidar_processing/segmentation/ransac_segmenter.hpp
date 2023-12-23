#ifndef LIDAR_PROCESSING__SEGMENTATION__RANSAC_SEGMENTER_HPP
#define LIDAR_PROCESSING__SEGMENTATION__RANSAC_SEGMENTER_HPP

#include "i_segmenter.hpp"

namespace lidar_processing::segmentation
{
class RansacSegmenter : public ISegmenter
{
  public:
    RansacSegmenter(/* constructor arguments */) : ISegmenter()
    {
    }

    void run() override;

  private:
};
} // namespace lidar_processing::segmentation

#endif // LIDAR_PROCESSING__SEGMENTATION__RANSAC_SEGMENTER_HPP
