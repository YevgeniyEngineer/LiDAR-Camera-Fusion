#include <utilities/thread_pool.hpp>

namespace utilities
{
ThreadPool::ThreadPool(std::size_t number_of_threads) : stop_(false)
{
    for (std::size_t i = 0; i < number_of_threads; ++i)
    {
        threads_.emplace_back([this]() {
            while (true)
            {
                std::packaged_task<void()> task;
                {
                    std::unique_lock<std::mutex> lock(mutex_);

                    condition_variable_.wait(lock, [this] { return stop_ || !tasks_.empty(); });

                    if (stop_)
                    {
                        return;
                    }

                    task = std::move(tasks_.front());
                    tasks_.pop();
                }

                task();
            }
        });
    }
}

ThreadPool::~ThreadPool()
{
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
        condition_variable_.notify_all();
    }

    for (auto &thread : threads_)
    {
        thread.join();
    }
}

std::future<void> ThreadPool::enqueue(std::function<void()> func)
{
    std::packaged_task<void()> task(func);
    std::future<void> result = task.get_future();

    {
        const std::lock_guard<std::mutex> lock(mutex_);

        if (stop_)
        {
            throw std::runtime_error("enqueue on stopped ThreadPool");
        }

        tasks_.emplace(std::move(task));
    }

    condition_variable_.notify_one();
    return result;
}
} // namespace utilities
