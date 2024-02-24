#include <utilities_lib/fifo_queue.hpp>

#include <gtest/gtest.h>

using namespace utilities_lib;

// Test default constructor
TEST(FIFOQueueTest, DefaultConstructor)
{
    FIFOQueue<int> queue;
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0);
}

// Test constructor with capacity
TEST(FIFOQueueTest, ConstructorWithCapacity)
{
    FIFOQueue<int> queue(10); // Capacity of 10
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0);
}

// Test push and pop functionality
TEST(FIFOQueueTest, PushAndPop)
{
    FIFOQueue<int> queue(5);
    queue.push(1);
    queue.push(2);

    EXPECT_FALSE(queue.empty());
    EXPECT_EQ(queue.size(), 2);
    EXPECT_EQ(queue.front(), 1);

    queue.pop();
    EXPECT_EQ(queue.size(), 1);
    EXPECT_EQ(queue.front(), 2);

    queue.pop();
    EXPECT_TRUE(queue.empty());
}

// Test the full condition
TEST(FIFOQueueTest, FullQueue)
{
    FIFOQueue<int> queue(2);
    queue.push(1);
    queue.push(2);

    EXPECT_TRUE(queue.full());
    EXPECT_EQ(queue.size(), 2);
}

// Test the resize functionality
TEST(FIFOQueueTest, Reserve)
{
    FIFOQueue<int> queue(2);
    queue.push(1);
    queue.push(2);

    queue.reserve(4);
    EXPECT_EQ(queue.size(), 2);
    EXPECT_FALSE(queue.full());

    queue.push(3);
    EXPECT_EQ(queue.size(), 3);
}

// Test clear functionality
TEST(FIFOQueueTest, Clear)
{
    FIFOQueue<int> queue;
    queue.push(1);
    queue.push(2);

    queue.clear();
    EXPECT_TRUE(queue.empty());
    EXPECT_EQ(queue.size(), 0);
}

// Test wrap-around functionality
TEST(FIFOQueueTest, WrapAround)
{
    FIFOQueue<int> queue(3);
    queue.push(1);
    queue.push(2);
    queue.pop(); // Remove 1
    queue.push(3);
    queue.push(4); // This should wrap around

    EXPECT_EQ(queue.front(), 2);
    queue.pop(); // Remove 2
    EXPECT_EQ(queue.front(), 3);
}
