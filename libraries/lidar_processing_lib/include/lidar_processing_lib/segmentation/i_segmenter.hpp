#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__I_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__I_SEGMENTER_HPP

#include <data_types_lib/segmentation_label.hpp> // SegmentationLabel

#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

#include <memory>      // std::unique_ptr, std::make_unique
#include <type_traits> // std::is_base_of_v
#include <utility>     // std::forward

namespace lidar_processing_lib::segmentation
{
class ISegmenter
{
  public:
    using UniquePtr = std::unique_ptr<ISegmenter>;
    using SegmentationLabel = data_types_lib::SegmentationLabel;

    /// @brief Deleted copy constructor.
    ISegmenter(const ISegmenter &) = delete;

    /// @brief Deleted copy assignment operator.
    ISegmenter &operator=(const ISegmenter &) = delete;

    /// @brief Deleted move constructor.
    ISegmenter(ISegmenter &&) = delete;

    /// @brief Deleted move assignment operator.
    ISegmenter &operator=(ISegmenter &&) = delete;

    /// @brief Default constructor.
    ISegmenter() = default;

    /// @brief Virtual destructor.
    virtual ~ISegmenter() = default;

    /// @brief Templated static factory method.
    template <typename DerivedSegmenter, typename... Args> static UniquePtr createUnique(Args &&...args)
    {
        static_assert(std::is_base_of_v<ISegmenter, DerivedSegmenter>,
                      "DerivedSegmenter must be a derived class of ISegmenter");

        return std::make_unique<DerivedSegmenter>(std::forward<Args>(args)...);
    }

    /// @brief Pure virtual run method to be implemented by the derived class.
    /// @param cloud - Input variant of the point cloud.
    /// @param labels - Output segmentation labels (equal to the number of elements in the input cloud).
    virtual void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels) = 0;

    /// @brief Pure virtual run method to be implemented by the derived class.
    /// @param cloud - Input variant of the point cloud.
    /// @param labels - Output segmentation labels (equal to the number of elements in the input cloud).
    virtual void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels) = 0;
};
} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING_LIB__SEGMENTATION__I_SEGMENTER_HPP
