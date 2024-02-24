#pragma once

#include <cstdint> // std::size_t
#include <new>     // ::new
#include <utility> // std::move

namespace utilities_lib
{
template <typename T> class FIFOQueue final
{
  public:
    /// @brief A non-default constructor that also preallocates memory for the internal buffer
    explicit FIFOQueue(const std::size_t capacity) : capacity_{capacity}, head_{0}, tail_{0}, count_{0}
    {
        buffer_ = static_cast<T *>(::operator new(capacity * sizeof(T)));
    }

    /// @brief Default constructor creates a 0 element buffer
    FIFOQueue() : FIFOQueue(0)
    {
    }

    /// @brief Destructor - cleans any remaining memory from the buffer
    ~FIFOQueue() noexcept
    {
        clear();
        ::operator delete(buffer_);
    }

    /// @brief Preallocates or reallocates memory up to specified capacity for the internal buffer
    inline void reserve(const std::size_t new_capacity)
    {
        if (new_capacity <= capacity_)
        {
            return; // Enough storage
        }

        T *new_buffer = static_cast<T *>(::operator new(new_capacity * sizeof(T)));
        for (std::size_t i = 0; i < count_; ++i)
        {
            std::size_t index = head_ + i;
            while (index >= capacity_)
            {
                index -= capacity_;
            }

            ::new (&new_buffer[i]) T{std::move(buffer_[index])};
            buffer_[index].~T();
        }

        ::operator delete(buffer_);
        buffer_ = new_buffer;
        capacity_ = new_capacity;
        head_ = 0;
        tail_ = count_;
    }

    /// @brief Checks if the queue is empty
    inline bool empty() const noexcept
    {
        return (count_ == 0);
    }

    /// @brief Checks if the queue is full
    inline bool full() const noexcept
    {
        return (count_ == capacity_);
    }

    /// @brief Retrieves current size of the queue
    inline std::size_t size() const noexcept
    {
        return count_;
    }

    /// @brief Adds element to the end of the queue
    /// @param value const l-value reference
    inline void push(const T &value)
    {
        emplace(value);
    }

    /// @brief Adds element to the end of the queue
    /// @param value r-value reference
    inline void push(T &&value)
    {
        emplace(std::move(value));
    }

    /// @brief Adds element to the end of the queue
    /// @tparam ...Args Template types of the parameter pack
    /// @param ...args Parameter pack used to construct object of type T
    template <typename... Args> inline void emplace(Args &&...args)
    {
        if (full())
        {
            // Perform reallocation here - double the old capacity
            reserve(capacity_ * 2);
        }

        ::new (&buffer_[tail_]) T{std::forward<Args>(args)...};

        ++tail_;
        if (tail_ >= capacity_)
        {
            tail_ = 0;
        }

        ++count_;
    }

    /// @brief Removes first element from the queue
    inline void pop() noexcept
    {
        if (!empty())
        {
            buffer_[head_].~T();

            ++head_;
            if (head_ >= capacity_)
            {
                head_ = 0;
            }

            --count_;
        }
    }

    /// @brief Returns first added element to the queue
    inline T &front() noexcept
    {
        return buffer_[head_];
    }

    /// @brief Returns first added element to the queue
    inline const T &front() const noexcept
    {
        return buffer_[head_];
    }

    /// @brief Returns last added element to the queue
    inline T &back() noexcept
    {
        return buffer_[((tail_ == 0) ? capacity_ : tail_) - 1];
    }

    /// @brief Returns last added element to the queue
    inline const T &back() const noexcept
    {
        return buffer_[((tail_ == 0) ? capacity_ : tail_) - 1];
    }

    /// @brief Clear all elements from the queue
    inline void clear() noexcept
    {
        while (!empty())
        {
            pop();
        }
    }

  private:
    T *buffer_;
    std::size_t capacity_;
    std::size_t head_;
    std::size_t tail_;
    std::size_t count_;
};
} // namespace utilities_lib
