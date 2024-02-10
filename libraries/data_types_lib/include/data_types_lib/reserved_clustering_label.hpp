#ifndef DATA_TYPES_LIB__RESERVED_CLUSTERING_LABEL_LABEL_HPP
#define DATA_TYPES_LIB__RESERVED_CLUSTERING_LABEL_LABEL_HPP

#include <cstdint>

namespace data_types_lib
{
using ClusteringLabel = std::int32_t;

enum class ReservedClusteringLabel : std::int32_t
{
    UNKNOWN = -2,
    OUTLIER = -1
};
} // namespace data_types_lib

#endif // DATA_TYPES_LIB__RESERVED_CLUSTERING_LABEL_LABEL_HPP
