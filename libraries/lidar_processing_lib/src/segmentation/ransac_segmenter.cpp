#include <lidar_processing_lib/segmentation/ransac_segmenter.hpp>

namespace lidar_processing_lib::segmentation
{
RansacSegmenter::RansacSegmenter(float height_offset, float orthogonal_distance_threshold,
                                 std::uint32_t number_of_iterations, float max_plane_inclination_deg,
                                 float consideration_radius, float consideration_height)
    : ISegmenter(), height_offset_(height_offset), orthogonal_distance_threshold_(orthogonal_distance_threshold),
      number_of_iterations_(number_of_iterations), max_plane_inclination_deg_(max_plane_inclination_deg),
      consideration_radius_(consideration_radius), consideration_height_(consideration_height)
{
    processing_points_.reserve(200'000U);
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
} // namespace lidar_processing_lib::segmentation
