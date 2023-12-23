#include <lidar_segmentation/ransac_segmenter.hpp>

namespace lidar_processing::lidar_segmentation
{
RansacSegmenter::RansacSegmenter(float orthogonal_distance_threshold, std::uint32_t number_of_iterations,
                                 std::size_t thread_count)
    : ISegmenter(), orthogonal_distance_threshold_(orthogonal_distance_threshold),
      number_of_iterations_(number_of_iterations), thread_pool_(thread_count), futures_(thread_count)
{
}

RansacSegmenter::~RansacSegmenter()
{
}

void RansacSegmenter::run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels)
{
    segment(cloud, labels);
}

void RansacSegmenter::run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels)
{
    segment(cloud, labels);
}
} // namespace lidar_processing::lidar_segmentation
