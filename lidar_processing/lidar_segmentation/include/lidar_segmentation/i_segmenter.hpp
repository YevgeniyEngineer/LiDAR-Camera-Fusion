#ifndef LIDAR_PROCESSING__LIDAR_SEGMENTATION__I_SEGMENTER_HPP
#define LIDAR_PROCESSING__LIDAR_SEGMENTATION__I_SEGMENTER_HPP

#include <memory>      // std::unique_ptr, std::make_unique
#include <type_traits> // std::is_base_of_v
#include <utility>     // std::forward

namespace lidar_processing::lidar_segmentation
{
class ISegmenter
{
  public:
    using UniquePtr = std::unique_ptr<ISegmenter>;

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

    /// @brief Templated static create method.
    template <typename DerivedSegmenter, typename... Args> static UniquePtr createUnique(Args &&...args)
    {
        static_assert(std::is_base_of_v<ISegmenter, DerivedSegmenter>,
                      "DerivedSegmenter must be a derived class of ISegmenter");

        return std::make_unique<DerivedSegmenter>(std::forward<Args>(args)...);
    }

    /// @brief Pure virtual run method to be implemented by the derived class.
    virtual void run() = 0;
};
} // namespace lidar_processing::lidar_segmentation

#endif // LIDAR_PROCESSING__LIDAR_SEGMENTATION__I_SEGMENTER_HPP
