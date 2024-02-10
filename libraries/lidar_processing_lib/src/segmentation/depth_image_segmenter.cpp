#include <lidar_processing_lib/segmentation/depth_image_segmenter.hpp>

namespace lidar_processing_lib::segmentation
{
DepthImageSegmenter::DepthImageSegmenter() : range_image_(RANGE_IMAGE_HEIGHT * RANGE_IMAGE_WIDTH)
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
