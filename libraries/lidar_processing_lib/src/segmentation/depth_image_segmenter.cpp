#include <lidar_processing_lib/segmentation/depth_image_segmenter.hpp>

namespace lidar_processing_lib::segmentation
{
DepthImageSegmenter::DepthImageSegmenter(float min_range, float max_range)
    : min_range_(min_range), max_range_(max_range), depth_image_(DEPTH_IMAGE_HEIGHT * DEPTH_IMAGE_WIDTH)
{
}

DepthImageSegmenter::~DepthImageSegmenter()
{
}

void DepthImageSegmenter::run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels)
{
    segment(cloud, labels);
}

void DepthImageSegmenter::run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels)
{
    segment(cloud, labels);
}
} // namespace lidar_processing_lib::segmentation
