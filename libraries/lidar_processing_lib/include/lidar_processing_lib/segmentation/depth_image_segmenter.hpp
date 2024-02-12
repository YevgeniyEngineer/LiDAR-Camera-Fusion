#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP

#include "i_segmenter.hpp"        // ISegmenter
#include <array>                  // std::array
#include <chrono>                 // std::chrono
#include <cstdint>                // std::uint32_t
#include <eigen3/Eigen/Dense>     // Eigen::
#include <limits>                 // std::numeric_limits
#include <utilities_lib/math.hpp> // constexprRound
#include <vector>                 // std::vector

namespace lidar_processing_lib::segmentation
{
// Elevation map
constexpr static inline std::uint32_t numberOfRingsInRingElevationMap(const float min_range, const float max_range,
                                                                      const float dR) noexcept
{
    return static_cast<std::uint32_t>(std::ceil((max_range - min_range) / dR));
}

class DepthImageSegmenter : public ISegmenter
{
  public:
    using SegmentationLabel = data_types_lib::SegmentationLabel;

    struct RangeImagePixel
    {
        // This pixel does not map to point cloud
        static constexpr auto INVALID_INDEX = std::numeric_limits<std::uint32_t>::max();

        float range = std::numeric_limits<float>::max();
        std::uint32_t index = INVALID_INDEX; // index of the point cloud
        std::uint8_t r = 0U;
        std::uint8_t g = 0U;
        std::uint8_t b = 0U;

        inline void reset() noexcept
        {
            range = std::numeric_limits<float>::max();
            index = INVALID_INDEX;
            r = 0U;
            g = 0U;
            b = 0U;
        }
    };

    // Values acceptable for Velodyne HDL-64E (tightest values)
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0F);

    static constexpr float VERTICAL_RESOLUTION_DEG = 0.33F;
    static constexpr float VERTICAL_RESOLUTION_RAD = VERTICAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float VERTICAL_FIELD_OF_VIEW_DEG = 26.9F;
    static constexpr float VERTICAL_FIELD_OF_VIEW_RAD = VERTICAL_FIELD_OF_VIEW_DEG * DEG_TO_RAD;

    static constexpr float MIN_ELEVATION_DEG = -24.8;
    static constexpr float MIN_ELEVATION_RAD = MIN_ELEVATION_DEG * DEG_TO_RAD;
    static constexpr float MAX_ELEVATION_DEG = MIN_ELEVATION_DEG + VERTICAL_FIELD_OF_VIEW_DEG;
    static constexpr float MAX_ELEVATION_RAD = MAX_ELEVATION_DEG * DEG_TO_RAD;

    static constexpr float HORIZONTAL_RESOLUTION_DEG = 0.1F;
    static constexpr float HORIZONTAL_RESOLUTION_RAD = HORIZONTAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_DEG = 360.0F;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_RAD = HORIZONTAL_FIELD_OF_VIEW_DEG * DEG_TO_RAD;

    // width ~= 3600
    static constexpr auto DEPTH_IMAGE_WIDTH =
        static_cast<std::uint32_t>(std::ceil(HORIZONTAL_FIELD_OF_VIEW_DEG / HORIZONTAL_RESOLUTION_DEG));

    // height ~= 82
    static constexpr auto DEPTH_IMAGE_HEIGHT =
        static_cast<std::uint32_t>(std::ceil(VERTICAL_FIELD_OF_VIEW_DEG / VERTICAL_RESOLUTION_DEG));

    // Elevation map parameters
    static constexpr float MIN_DISTANCE_M = 0.0F;
    static constexpr float MAX_DISTANCE_M = 80.0F;
    static constexpr float RING_SPACING_M = 1.5F;

    static constexpr std::uint32_t NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP =
        numberOfRingsInRingElevationMap(MIN_DISTANCE_M, MAX_DISTANCE_M, RING_SPACING_M);

    static constexpr std::uint32_t NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP = 24U;

    static constexpr std::uint32_t MAX_CLOUD_POINTS = 350000U;

    static constexpr auto MAX_POINTS_PER_CELL_RING_ELEVATION_CONJUNCTION_MAP = static_cast<std::uint32_t>(
        std::ceil(static_cast<float>(MAX_CLOUD_POINTS) / (NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP *
                                                          NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP)));

    DepthImageSegmenter(float min_range = MIN_DISTANCE_M, float max_range = MAX_DISTANCE_M);
    ~DepthImageSegmenter();

    void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels) override;
    void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels) override;

  private:
    float min_range_;
    float max_range_;

    float dH_ = 0.20F;
    float dR_ = 1.50F;
    float dM_ = 0.15F;

    // For embedding into polar grid (Ring Elevation Conjunction Map)
    struct PointXYZIL final
    {
        float x;
        float y;
        float z;
        std::uint32_t index;
        SegmentationLabel label;

        PointXYZIL() = default;

        PointXYZIL(float x, float y, float z, std::uint32_t index, SegmentationLabel label)
            : x(x), y(y), z(z), index(index), label(label)
        {
        }
    };

    // Range image - contiguous in memory, index mapping [width x height] - row major order
    std::vector<RangeImagePixel> depth_image_;

    // Stores min elevation values in the ring elevation map
    std::array<float,
               NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP * NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP>
        ring_elevation_conjunction_map_;

    std::array<std::vector<PointXYZIL>,
               NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP * NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP>
        polar_grid_;

    constexpr static inline std::uint32_t rowMajorIndexDepthImage(const std::uint32_t x, const std::uint32_t y) noexcept
    {
        return (y * DEPTH_IMAGE_WIDTH + x);
    }

    constexpr static inline std::uint32_t rowMajorIndexRingElevationConjunctionMap(
        const std::uint32_t channel_index, const std::uint32_t ring_index) noexcept
    {
        return (channel_index * NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP + ring_index);
    }

    inline void resetRingElevationConjunctionMap() noexcept
    {
        for (auto &cell : ring_elevation_conjunction_map_)
        {
            cell = std::numeric_limits<float>::max();
        }
    }

    template <typename PointT> void embedCloudIntoPolarGrid(const pcl::PointCloud<PointT> &cloud);

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels);

    template <typename PointT> static inline float rangeSquared(const PointT &point) noexcept
    {
        return (point.x * point.x) + (point.y * point.y);
    }

    template <typename PointT> static inline float range(const PointT &point) noexcept
    {
        return std::sqrt(rangeSquared(point));
    }

    template <typename PointT> void constructDepthImage(const pcl::PointCloud<PointT> &cloud);

    void segmentRingElevationConjunctionMap();
};

template <typename PointT> void DepthImageSegmenter::embedCloudIntoPolarGrid(const pcl::PointCloud<PointT> &cloud)
{
    static constexpr auto CHANNEL_RESOLUTION_RAD =
        static_cast<float>(2.0 * M_PI / NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP);

    // Clear old slices
    for (auto &slice : polar_grid_)
    {
        slice.clear();
    }

    // Embed points into each channel
    for (std::uint32_t i = 0U; i < cloud.points.size(); ++i)
    {
        const auto &point = cloud.points[i];

        // Calculate distance from sensor
        const float distance = std::sqrt((point.x * point.x) + (point.y * point.y));

        if ((distance > MIN_DISTANCE_M) && (distance <= MAX_DISTANCE_M))
        {
            // Convert azimuth angle to degrees and shift range to [0, 360) OR [0, 2 * PI]
            float azimuth_rad = utilities_lib::atan2Approx(point.y, point.x);

            // Adjust to range [0, 2 * pi]
            if (azimuth_rad < 0)
            {
                azimuth_rad += static_cast<float>(2.0 * M_PI);
            }

            // Determine channel index using floor division
            const std::uint32_t channel_index =
                std::min(static_cast<std::uint32_t>(azimuth_rad / CHANNEL_RESOLUTION_RAD),
                         (NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP - 1U));

            // Determine ring index
            const std::uint32_t ring_index =
                std::min(static_cast<std::uint32_t>((distance - MIN_DISTANCE_M) / RING_SPACING_M),
                         (NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP - 1U));

            // Embed point into polar grid channel and bin (ring)
            polar_grid_[rowMajorIndexRingElevationConjunctionMap(channel_index, ring_index)].emplace_back(
                point.x, point.y, point.z, i, SegmentationLabel::UNKNOWN);
        }
    }

    // Sort points in increasing radial order, in each channel in each ring
    for (std::uint32_t channel_index = 0U; channel_index < NUMBER_OF_CHANNELS_IN_RING_ELEVATION_CONJUNCTION_MAP;
         ++channel_index)
    {
        for (std::uint32_t ring_index = 0U; ring_index < NUMBER_OF_RINGS_IN_RING_ELEVATION_CONJUNCTION_MAP;
             ++ring_index)
        {
            const std::uint32_t cell_index = rowMajorIndexRingElevationConjunctionMap(channel_index, ring_index);
            auto &cell = polar_grid_[cell_index];

            if (!cell.empty())
            {
                std::sort(cell.begin(), cell.end(), [](const auto &p1, const auto &p2) noexcept {
                    const float dr1 = (p1.x * p1.x) + (p1.y * p1.y);
                    const float dr2 = (p2.x * p2.x) + (p2.y * p2.y);
                    return dr1 < dr2;
                });
            }
        }
    }
}

template <typename PointT> void DepthImageSegmenter::constructDepthImage(const pcl::PointCloud<PointT> &cloud)
{
    static constexpr float ELEVATION_MID_RAD = (MIN_ELEVATION_RAD + MAX_ELEVATION_RAD) / 2.0F;

    // Ensure the range image is properly sized and reset
    if (depth_image_.size() != (DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT))
    {
        depth_image_.resize(DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT);
    }

    // Reset all pixels of the range image
    std::fill(depth_image_.begin(), depth_image_.end(), RangeImagePixel());

    // Fill range image with point cloud points
    for (std::uint32_t i = 0U; i < cloud.points.size(); ++i)
    {
        const auto &point = cloud.points[i];

        const float dd_sqr = point.x * point.x + point.y * point.y;
        const float range = std::sqrt(dd_sqr + point.z * point.z);

        if ((range < min_range_) || (range > max_range_))
        {
            continue; // Skip points out of range
        }

        const float azimuth_rad = std::atan2(point.y, point.x);
        const float elevation_rad = std::atan2(point.z, std::sqrt(dd_sqr));

        // Convention: (0, 0) coordinate of the image located at the top left corner

        // Adjust azimuth to map zero azimuth to the center of the image width
        const auto x = static_cast<std::int32_t>(((azimuth_rad / M_PIf32) * 0.5F + 0.5F) * DEPTH_IMAGE_WIDTH);

        // Ensure x is within the valid range
        if ((x < 0) || (x > static_cast<std::int32_t>(DEPTH_IMAGE_WIDTH - 1)))
        {
            continue;
        }

        // Adjust elevation to map zero elevation to the center of the image height
        const auto y = static_cast<std::int32_t>(
            ((elevation_rad - ELEVATION_MID_RAD) / (MAX_ELEVATION_RAD - MIN_ELEVATION_RAD) + 0.5F) *
            DEPTH_IMAGE_HEIGHT);

        // Ensure y is within the valid range
        if ((y < 0) || (y > static_cast<std::int32_t>(DEPTH_IMAGE_HEIGHT - 1)))
        {
            continue;
        }

        const auto row_major_index = rowMajorIndexDepthImage(x, y);

        auto &pixel = depth_image_[row_major_index];
        if (range < pixel.range)
        {
            pixel.range = range;
            pixel.index = i;
            pixel.r = 0;
            pixel.g = 255;
            pixel.b = 0;
        }
    }

    std::uint32_t number_of_depth_image_points = 0U;
    for (const auto &point : depth_image_)
    {
        if (point.index != RangeImagePixel::INVALID_INDEX)
        {
            ++number_of_depth_image_points;
        }
    }

    std::cerr << "Number of depth image pixels: " << number_of_depth_image_points << " out of a total of "
              << cloud.points.size() << " points" << std::endl;
}

void DepthImageSegmenter::segmentRingElevationConjunctionMap()
{
    // Course segmentation algorithm
    // Input: Raw Point Cloud
    // Output: First Labelled Point Cloud
    // 1. Choose ground height threshold, dH
    // 2. Choose grid ring spacing, dR
    // 3. Choose road maximum slope, dM
    //
    // For each Point(t,c) in Cloud:
    //      Label(t,c) = GROUND
    //      Point(t,c) in GRID(m,n)
    //      Elevation(m,n) = min(Elevation(m,n), z(t,c))
    //
    // For each Elevation(m,n) in ElevationMap:
    //      Elevation(m,n) = min(Elevation(m,n), Elevation(m-1,n) + dR * tan(dM))
    //
    // For each Point(t,c) in Cloud:
    //      Point(t,c) in GRID(m,n)
    //      if z(t,c) >= Elevation(m,n) + dH:
    //          Label(t,c) = OBSTACLE
}

template <typename PointT>
void DepthImageSegmenter::segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels)
{
    // Set all labels to unknown
    labels.assign(cloud.points.size(), SegmentationLabel::UNKNOWN);

    // Construct depth image
    const auto t1 = std::chrono::steady_clock::now();

    constructDepthImage(cloud);

    const auto t2 = std::chrono::steady_clock::now();
    std::cerr << "Construction of depth image time [ms]: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
}

} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP
