#include <utilities/bounded_vector.hpp>

#include <gtest/gtest.h>

using namespace utilities;

/// @brief Test fixture for BoundedVector
class BoundedVectorTest : public ::testing::Test
{
  protected:
    BoundedVector<double, 10> vector;
};

// Test the default constructor
TEST_F(BoundedVectorTest, DefaultConstructor)
{
    EXPECT_EQ(vector.size(), 0U);
}

// Test the push_back and pop_back methods
TEST_F(BoundedVectorTest, PushAndPopBack)
{
    vector.push_back(1.0);
    EXPECT_EQ(vector.size(), 1);
    vector.pop_back();
    EXPECT_EQ(vector.size(), 0);
}

// Test the copy constructor
TEST_F(BoundedVectorTest, CopyConstructor)
{
    vector.push_back(1.0);
    BoundedVector<double, 10> vector_copy(vector);
    EXPECT_EQ(vector_copy.size(), vector.size());
}

// Test the move constructor
TEST_F(BoundedVectorTest, MoveConstructor)
{
    vector.push_back(1.0);
    BoundedVector<double, 10> vector_moved(std::move(vector));
    EXPECT_EQ(vector_moved.size(), 1);
    EXPECT_EQ(vector.size(), 0); // Vector has been moved
}

// Test the copy assignment operator
TEST_F(BoundedVectorTest, CopyAssignment)
{
    vector.push_back(1.0);
    BoundedVector<double, 10> vector_copy;
    vector_copy = vector;
    EXPECT_EQ(vector_copy.size(), vector.size());
}

// Test the move assignment operator
TEST_F(BoundedVectorTest, MoveAssignment)
{
    vector.push_back(1.0);
    BoundedVector<double, 10> vector_moved;
    vector_moved = std::move(vector);
    EXPECT_EQ(vector_moved.size(), 1);
    EXPECT_EQ(vector.size(), 0); // Vector has been moved
}

// Test forward iterator
TEST_F(BoundedVectorTest, ForwardIterator)
{
    vector.push_back(1.0);
    vector.push_back(2.0);
    auto it = vector.begin();
    EXPECT_DOUBLE_EQ(*it, 1.0);
    ++it;
    EXPECT_DOUBLE_EQ(*it, 2.0);
}

// Test const forward iterator
TEST_F(BoundedVectorTest, ConstForwardIterator)
{
    vector.push_back(1.0);
    vector.push_back(2.0);
    const auto &const_vector = vector;
    auto it = const_vector.cbegin();
    EXPECT_DOUBLE_EQ(*it, 1.0);
    ++it;
    EXPECT_DOUBLE_EQ(*it, 2.0);
}

// Test reverse iterator
TEST_F(BoundedVectorTest, ReverseIterator)
{
    vector.push_back(1.0);
    vector.push_back(2.0);
    auto it = vector.rbegin();
    EXPECT_DOUBLE_EQ(*it, 2.0);
    ++it;
    EXPECT_DOUBLE_EQ(*it, 1.0);
}

// Test const reverse iterator
TEST_F(BoundedVectorTest, ConstReverseIterator)
{
    vector.push_back(1.0);
    vector.push_back(2.0);
    const auto &const_vector = vector;
    auto it = const_vector.crbegin();
    EXPECT_DOUBLE_EQ(*it, 2.0);
    ++it;
    EXPECT_DOUBLE_EQ(*it, 1.0);
}

// Test operator[] for access and modification
TEST_F(BoundedVectorTest, OperatorSquareBrackets)
{
    vector.push_back(1.0);
    vector.push_back(2.0);

    // Test reading values
    EXPECT_DOUBLE_EQ(vector[0], 1.0);
    EXPECT_DOUBLE_EQ(vector[1], 2.0);

    // Test modifying values
    vector[0] = 3.0;
    vector[1] = 4.0;
    EXPECT_DOUBLE_EQ(vector[0], 3.0);
    EXPECT_DOUBLE_EQ(vector[1], 4.0);
}

// Test at() method for access and exception handling
TEST_F(BoundedVectorTest, AtMethod)
{
    vector.push_back(1.0);
    vector.push_back(2.0);

    // Test reading values
    EXPECT_DOUBLE_EQ(vector.at(0), 1.0);
    EXPECT_DOUBLE_EQ(vector.at(1), 2.0);

    // Test modifying values
    vector.at(0) = 3.0;
    vector.at(1) = 4.0;
    EXPECT_DOUBLE_EQ(vector.at(0), 3.0);
    EXPECT_DOUBLE_EQ(vector.at(1), 4.0);
    EXPECT_DOUBLE_EQ(vector.front(), 3.0);
    EXPECT_DOUBLE_EQ(vector.back(), 4.0);

    // Test out-of-bounds access
    EXPECT_THROW(vector.at(2), std::out_of_range);
}

// int main(int argc, char *argv[])
// {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
