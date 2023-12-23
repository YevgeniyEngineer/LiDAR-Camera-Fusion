#ifndef UTILITIES__THREAD_POOL_HPP
#define UTILITIES__THREAD_POOL_HPP

#include <condition_variable> // std::condition_variable
#include <cstdlib>            // std::size_t
#include <functional>         // std::function
#include <future>             // std::packaged_task
#include <mutex>              // std::mutex
#include <queue>              // std::queue
#include <thread>             // std::thread
#include <utility>            // std::move
#include <vector>             // std::vector

namespace utilities
{
class ThreadPool final
{
  public:
    /// @brief Deleted default constructor.
    ThreadPool() = delete;

    /// @brief Non-default constructor.
    explicit ThreadPool(std::size_t number_of_threads);

    /// @brief Destructor.
    ~ThreadPool();

    /// @brief Deleted copy constructor.
    ThreadPool(const ThreadPool &) = delete;

    /// @brief Deleted copy assignment operator.
    ThreadPool &operator=(const ThreadPool &) = delete;

    /// @brief Deleted move constructor.
    ThreadPool(ThreadPool &&) = delete;

    /// @brief Deleted move assignment operator.
    ThreadPool &operator=(ThreadPool &&) = delete;

    /// @brief Enqueue a task into a thread pool.
    /// @param Function to be enqueued.
    /// @return Future.
    std::future<void> enqueue(std::function<void()> func);

  private:
    std::vector<std::thread> threads_;
    std::queue<std::packaged_task<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable condition_variable_;
    bool stop_;
};
} // namespace utilities

#endif // UTILITIES__THREAD_POOL_HPP
