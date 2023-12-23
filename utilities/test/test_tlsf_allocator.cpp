#include <utilities/tlsf_allocator.hpp>

#include <gtest/gtest.h>

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

using namespace utilities;

static std::int64_t allocated_elements = 0;

struct ComplexData
{
    int id;
    std::string name;
    std::vector<double> values;

    ComplexData()
    {
        ++allocated_elements;
    }

    ComplexData(int id, const std::string &name, const std::vector<double> &values) : id(id), name(name), values(values)
    {
        ++allocated_elements;
    }

    ~ComplexData()
    {
        --allocated_elements;
    }

    // Equality operator for testing purposes
    bool operator==(const ComplexData &other) const
    {
        return (id == other.id) && (name == other.name) && (values == other.values);
    }
};

// Test Fixture for TLSFAllocator tests
class TLSFAllocatorTest : public ::testing::Test
{
  protected:
    std::vector<ComplexData, TLSFAllocator<ComplexData>> test_vector_;

    void SetUp() override
    {
        // Initial setup can be done here
    }

    void TearDown() override
    {
        // Cleanup can be done here
    }
};

// Test case for isolated allocator test
TEST_F(TLSFAllocatorTest, SimpleAllocation)
{
    TLSFAllocator<int> allocator;
    int *ptr = allocator.allocate(1); // Allocate space for one int
    ASSERT_NE(ptr, nullptr);          // Check if allocation was successful

    allocator.construct(ptr, 123); // Construct an integer
    EXPECT_EQ(*ptr, 123);          // Check if the value is correct

    allocator.destroy(ptr);       // Destroy the integer
    allocator.deallocate(ptr, 1); // Deallocate space
}

// Test case for adding a large number of elements to the vector
TEST_F(TLSFAllocatorTest, AddManyElementsWithReserve)
{
    // Reserve memory
    test_vector_.reserve(100000);

    // Add initial elements
    test_vector_.emplace_back(1, "Data1", std::vector<double>{1.0, 2.0, 3.0});
    test_vector_.emplace_back(2, "Data2", std::vector<double>{4.0, 5.0, 6.0});

    // Check initial conditions
    EXPECT_EQ(test_vector_.size(), 2);
    EXPECT_EQ(test_vector_[0], (ComplexData{1, "Data1", {1.0, 2.0, 3.0}}));
    EXPECT_EQ(test_vector_[1], (ComplexData{2, "Data2", {4.0, 5.0, 6.0}}));

    // Add 99,998 more elements
    for (std::size_t i = 3; i <= 100000; ++i)
    {
        test_vector_.emplace_back(i, "Data" + std::to_string(i), std::vector<double>{1.0 * i, 2.0 * i, 3.0 * i});
    }

    // Verify the final size
    EXPECT_EQ(test_vector_.size(), 100000);
    EXPECT_EQ(allocated_elements, 100000);

    // Optionally, verify some elements from the vector
    if (!test_vector_.empty())
    {
        EXPECT_EQ(test_vector_.front(), (ComplexData{1, "Data1", {1.0, 2.0, 3.0}}));
        EXPECT_EQ(test_vector_.back(), (ComplexData{100000, "Data100000", {100000.0, 200000.0, 300000.0}}));
    }

    test_vector_.clear();
    EXPECT_EQ(test_vector_.size(), 0);
    EXPECT_EQ(allocated_elements, 0);
}
