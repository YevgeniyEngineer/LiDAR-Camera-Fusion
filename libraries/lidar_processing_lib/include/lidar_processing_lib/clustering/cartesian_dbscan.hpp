#ifndef LIDAR_PROCESSING_LIB__CLUSTERING__CARTESIAN_DBSCAN_HPP
#define LIDAR_PROCESSING_LIB__CLUSTERING__CARTESIAN_DBSCAN_HPP

#include "i_clusterer.hpp" // IClusterer

namespace lidar_processing_lib::clustering
{
class CartesianDBSCAN : public IClusterer
{
  public:
    CartesianDBSCAN();
    ~CartesianDBSCAN();

    void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<ClusteringLabel> &labels) override;
    void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<ClusteringLabel> &labels) override;

  private:
    template <typename PointT> void cluster(const pcl::PointCloud<PointT> &cloud, std::vector<ClusteringLabel> &labels);
};

template <typename PointT>
void CartesianDBSCAN::cluster(const pcl::PointCloud<PointT> &cloud, std::vector<ClusteringLabel> &labels)
{
    // Reset labels
    labels.assign(cloud.points.size(), static_cast<ClusteringLabel>(ReservedClusteringLabel::UNKNOWN));

    // TODO: Perform clustering
}

} // namespace lidar_processing_lib::clustering

#endif // LIDAR_PROCESSING_LIB__CLUSTERING__CARTESIAN_DBSCAN_HPP
