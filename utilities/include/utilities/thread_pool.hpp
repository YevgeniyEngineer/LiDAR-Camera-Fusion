#ifndef UTILITIES__THREAD_POOL_HPP
#define UTILITIES__THREAD_POOL_HPP

#include <condition_variable> // std::condition_variable
#include <cstdlib>            // std::size_t
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

    /// @brief Type-erased enqueue of a task into a thread pool.
    /// @param Function to be enqueued.
    /// @return Future.
    template <class F> auto enqueue(F &&func) -> std::future<decltype(func())>
    {
        using return_type = decltype(func());
        std::packaged_task<return_type()> task(std::forward<F>(func));
        std::future<return_type> result = task.get_future();

        {
            const std::lock_guard<std::mutex> lock(mutex_);

            if (stop_)
            {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }

            tasks_.emplace([t = std::move(task)]() mutable { t(); });
        }

        condition_variable_.notify_one();
        return result;
    }

    /// @brief Get the number of running threads.
    inline std::size_t threadCount() const noexcept
    {
        return threads_.size();
    }

  private:
    std::vector<std::thread> threads_;
    std::queue<std::packaged_task<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable condition_variable_;
    bool stop_;
};
} // namespace utilities

#endif // UTILITIES__THREAD_POOL_HPP
