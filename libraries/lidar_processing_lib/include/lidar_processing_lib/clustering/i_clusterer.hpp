#ifndef LIDAR_PROCESSING_LIB__CLUSTERING__I_CLUSTER_HPP
#define LIDAR_PROCESSING_LIB__CLUSTERING__I_CLUSTER_HPP

#include <data_types_lib/reserved_clustering_label.hpp> // ReservedClusteringLabel

#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

#include <memory>      // std::unique_ptr
#include <type_traits> // std::is_base_of_v
#include <utility>     // std::forward

namespace lidar_processing_lib::clustering
{
class IClusterer
{
  public:
    using UniquePtr = std::unique_ptr<IClusterer>;

    /// @brief Deleted copy constructor.
    IClusterer(const IClusterer &) = delete;

    /// @brief Deleted copy assignment operator.
    IClusterer &operator=(const IClusterer &) = delete;

    /// @brief Deleted move constructor.
    IClusterer(IClusterer &&) = delete;

    /// @brief Deleted move assignment operator.
    IClusterer &operator=(IClusterer &&) = delete;

    /// @brief Default constructor.
    IClusterer() = default;

    /// @brief Virtual destructor.
    virtual ~IClusterer() = default;

    /// @brief Templated static factory method.
    template <typename DerivedClusterer, typename... Args> static UniquePtr createUnique(Args &&...args)
    {
        static_assert(std::is_base_of_v<IClusterer, DerivedClusterer>,
                      "DerivedClusterer must be a derived class of IClusterer");

        return std::make_unique<DerivedClusterer>(std::forward<Args>(args)...);
    }

    /// @brief Pure virtual run method to be implemented by the derived class.
    /// @param cloud - Input variant of the point cloud.
    /// @param labels - Output clustering labels (equal to the number of elements in the input cloud).
    virtual void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<ClusteringLabel> &labels) = 0;

    /// @brief Pure virtual run method to be implemented by the derived class.
    /// @param cloud - Input variant of the point cloud.
    /// @param labels - Output clustering labels (equal to the number of elements in the input cloud).
    virtual void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<ClusteringLabel> &labels) = 0;
};
} // namespace lidar_processing_lib::clustering

#endif // LIDAR_PROCESSING_LIB__CLUSTERING__I_CLUSTER_HPP
