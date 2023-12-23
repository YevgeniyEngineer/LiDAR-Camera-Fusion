#include <gtest/gtest.h>

#include <utilities/thread_pool.hpp>

#include <atomic>
#include <chrono>

using namespace utilities;

class ThreadPoolTest : public ::testing::Test
{
  protected:
    ThreadPoolTest()
    {
        // Initialization code here
    }

    ~ThreadPoolTest() override
    {
        // Cleanup any remaining resources
    }

    // Additional setup can go here
};

TEST_F(ThreadPoolTest, TaskExecution)
{
    ThreadPool pool(2); // Create a thread pool with 2 threads

    std::atomic<int> counter = 0;
    auto task = [&counter]() { counter++; };

    // Enqueue tasks
    auto future1 = pool.enqueue(task);
    auto future2 = pool.enqueue(task);

    // Wait for tasks to complete
    future1.get();
    future2.get();

    // Check if tasks were executed
    EXPECT_EQ(counter, 2);
}

TEST_F(ThreadPoolTest, DestructorBehavior)
{
    {
        ThreadPool pool(2);
        pool.enqueue([]() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
    } // Destructor called here

    // No direct way to test destructor behavior, but reaching here without
    // a crash or deadlock indicates that the destructor likely behaved correctly.
}
