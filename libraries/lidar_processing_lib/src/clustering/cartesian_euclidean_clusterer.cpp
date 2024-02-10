#include <lidar_processing_lib/clustering/cartesian_euclidean_clusterer.hpp>

namespace lidar_processing_lib::clustering
{
CartesianEuclideanClusterer::CartesianEuclideanClusterer()
{
}

CartesianEuclideanClusterer::~CartesianEuclideanClusterer()
{
}

void CartesianEuclideanClusterer::run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<ClusteringLabel> &labels)
{
    cluster(cloud, labels);
}

void CartesianEuclideanClusterer::run(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                      std::vector<ClusteringLabel> &labels)
{
    cluster(cloud, labels);
}
} // namespace lidar_processing_lib::clustering
