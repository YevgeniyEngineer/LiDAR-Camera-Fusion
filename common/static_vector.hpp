#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace common
{
template <typename T, std::size_t N> class StaticVector
{
    static_assert(N > 0, "Array of size 0 is not allowed.");

  public:
    using value_type = T;
    using pointer = T *;
    using const_pointer = const T *;
    using reference = T &;
    using const_reference = const T &;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    /// @throws std::bad_alloc if not enough memory for allocation
    constexpr StaticVector() : size_(0), data_(new T[N])
    {
    }

    /// @throws std::bad_alloc if not enough memory for allocation
    constexpr explicit StaticVector(size_type size) : size_(size <= N ? size : N), data_(new T[N])
    {
    }

    /// @throws std::bad_alloc if not enough memory for allocation
    constexpr explicit StaticVector(size_type size, const_reference value)
        : size_(size <= N ? size : N), data_(new T[N])
    {
        for (auto &element : data_)
        {
            element = value;
        }
    }

    /// @throws std::bad_alloc if not enough memory for allocation
    StaticVector(const StaticVector &other) : size_(other.size_), data_(new T[other.size_])
    {
        std::copy(other.data_, other.data_ + other.size_, data_);
    }

    /// @throws std::bad_alloc if not enough memory for allocation
    StaticVector &operator=(const StaticVector &other)
    {
        if (this != &other)
        {
            delete[] data_;
            size_ = other.size_;
            data_ = new T[N];
            std::copy(other.data_, other.data_ + other.size_, data_);
        }
        return *this;
    }

    StaticVector(StaticVector &&other) noexcept(std::is_nothrow_move_constructible_v<value_type>)
        : size_(other.size_), data_(std::exchange(other.data_, nullptr))
    {
    }

    StaticVector &operator=(StaticVector &&other) noexcept(std::is_nothrow_move_assignable_v<value_type>)
    {
        if (this != &other)
        {
            delete[] data_;
            size_ = other.size_;
            data_ = std::exchange(other.data_, nullptr);
        }
        return *this;
    }

    ~StaticVector()
    {
        delete[] data_;
    }

    template <typename Pointer, typename Reference> class RandomAccessIterator
    {
      public:
        using difference_type = std::ptrdiff_t;
        using value_type = T;
        using pointer = Pointer;
        using reference = Reference;
        using iterator_category = std::random_access_iterator_tag;

        constexpr explicit RandomAccessIterator(Pointer ptr) noexcept : ptr_(ptr)
        {
        }
        inline reference operator*() noexcept
        {
            return *ptr_;
        }
        inline pointer operator->() noexcept
        {
            return ptr_;
        }
        inline RandomAccessIterator &operator++() noexcept
        {
            ++ptr_;
            return *this;
        }
        inline RandomAccessIterator operator++(int) noexcept
        {
            RandomAccessIterator tmp(*this);
            ++ptr_;
            return tmp;
        }
        inline RandomAccessIterator &operator--() noexcept
        {
            --ptr_;
            return *this;
        }
        inline RandomAccessIterator operator--(int) noexcept
        {
            RandomAccessIterator tmp(*this);
            --ptr_;
            return tmp;
        }
        inline bool operator==(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ == rhs.ptr_;
        }
        inline bool operator!=(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ != rhs.ptr_;
        }
        /// @throws std::runtime_error if accessing an element outside of bounds
        inline RandomAccessIterator operator+(difference_type n) const
        {
            if (n >= N)
            {
                throw std::runtime_error("Attempting to access an element outside of bounds.");
            }
            return RandomAccessIterator(ptr_ + n);
        }
        /// @throws std::runtime_error if accessing an element outside of bounds
        inline RandomAccessIterator operator-(difference_type n) const noexcept
        {
            if (n >= N)
            {
                throw std::runtime_error("Attempting to access an element outside of bounds.");
            }
            return RandomAccessIterator(ptr_ - n);
        }
        inline difference_type operator-(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ - rhs.ptr_;
        }
        inline RandomAccessIterator &operator+=(difference_type n) noexcept
        {
            ptr_ += n;
            return *this;
        }
        inline RandomAccessIterator &operator-=(difference_type n) noexcept
        {
            ptr_ -= n;
            return *this;
        }
        inline bool operator<(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ < rhs.ptr_;
        }
        inline bool operator<=(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ <= rhs.ptr_;
        }
        inline bool operator>(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ > rhs.ptr_;
        }
        inline bool operator>=(const RandomAccessIterator &rhs) const noexcept
        {
            return ptr_ >= rhs.ptr_;
        }

      private:
        Pointer ptr_;
    };

    using iterator = RandomAccessIterator<T *, T &>;
    using const_iterator = RandomAccessIterator<const T *, const T &>;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    inline iterator begin() const noexcept
    {
        return iterator(data_);
    }
    inline iterator end() const noexcept
    {
        return iterator(data_ + size_);
    }
    inline const_iterator cbegin() const noexcept
    {
        return const_iterator(data_);
    }
    inline const_iterator cend() const noexcept
    {
        return const_iterator(data_ + size_);
    }
    inline reverse_iterator rbegin() noexcept
    {
        return reverse_iterator(end());
    }
    inline reverse_iterator rend() noexcept
    {
        return reverse_iterator(begin());
    }
    inline const_reverse_iterator rbegin() const noexcept
    {
        return const_reverse_iterator(end());
    }
    inline const_reverse_iterator rend() const noexcept
    {
        return const_reverse_iterator(begin());
    }
    inline const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator(cend());
    }
    inline const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator(cbegin());
    }

    inline bool empty() const noexcept
    {
        return (size_ == 0U);
    }
    inline size_type size() const noexcept
    {
        return size_;
    }
    constexpr static inline size_type capacity() noexcept
    {
        return N;
    }
    /// @throws std::bad_alloc if the new size is larger than allocated capacity
    void resize(size_type new_size)
    {
        if (new_size >= N)
        {
            throw std::bad_alloc("The new size is larger than allocated capacity.");
        }
        size_ = new_size;
    }
    /// @throws std::runtime_error if accessing an element outside of bounds
    reference at(size_type index)
    {
        if (index >= size_)
        {
            throw std::runtime_error("Attempting to access an element outside of bounds.");
        }
        return data_[index];
    }
    /// @throws std::runtime_error if accessing an element outside of bounds
    const_reference at(size_type index) const
    {
        if (index >= size_)
        {
            throw std::runtime_error("Attempting to access an element outside of bounds.");
        }
        return data_[index];
    }
    inline reference operator[](size_type index) noexcept
    {
        return data_[index];
    }
    inline const_reference operator[](size_type index) const noexcept
    {
        return data_[index];
    }
    /// @throws std::runtime_error if the data is empty
    reference front()
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("There are no elements in the static vector.");
        }
        return data_[0];
    }
    /// @throws std::runtime_error if the data is empty
    const_reference front() const
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("There are no elements in the static vector.");
        }
        return data_[0];
    }
    /// @throws std::runtime_error if the data is empty
    reference back()
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("There are no elements in the static vector.");
        }
        return data_[size - 1U];
    }
    /// @throws std::runtime_error if the data is empty
    const_reference back() const
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("There are no elements in the static vector.");
        }
        return data_[size - 1U];
    }
    inline pointer data() noexcept
    {
        return data_;
    }
    inline const_pointer data() const noexcept
    {
        return data_;
    }
    /// @throws std::runtime_error if no more elements are allowed to be added
    template <typename... Args> void emplace_back(Args &&...args)
    {
        if (size_ >= N)
        {
            throw std::runtime_error("Static vector reached maximum capacity.");
        }
        data_[size_++] = T(std::forward<Args>(args)...);
    }
    /// @throws std::runtime_error if no more elements are allowed to be added
    void push_back(const value_type &value)
    {
        if (size_ >= N)
        {
            throw std::runtime_error("Static vector reached maximum capacity.");
        }
        data_[size_++] = value;
    }
    /// @throws std::runtime_error if no more elements are allowed to be added
    void push_back(value_type &&value)
    {
        if (size_ >= N)
        {
            throw std::runtime_error("Static vector reached maximum capacity.");
        }
        data_[size_++] = std::move(value);
    }
    void pop_back()
    {
        if (size_ > 0U)
        {
            --size_;
        }
    }
    inline void clear() noexcept
    {
        size_ = 0U;
    }
    void swap(StaticVector &other) noexcept(std::is_nothrow_swappable_v<value_type>)
    {
        std::swap(data_, other.data_);
        std::swap(size_, other.size_);
    }

  private:
    std::size_t size_;
    T *data_;
};

template <typename T, std::size_t N> constexpr bool operator==(const StaticVector<T, N> &a, const StaticVector<T, N> &b)
{
    if (a.size() != b.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i)
    {
        if (!(a[i] == b[i]))
        {
            return false;
        }
    }
    return true;
}

template <typename T, std::size_t N> constexpr bool operator!=(const StaticVector<T, N> &a, const StaticVector<T, N> &b)
{
    if (a.size() != b.size())
    {
        return true;
    }
    for (std::size_t i = 0; i < a.size(); ++i)
    {
        if (!(a[i] == b[i]))
        {
            return true;
        }
    }
    return false;
}

template <typename T, std::size_t N> void swap(StaticVector<T, N> &a, StaticVector<T, N> &b)
{
    b.swap(a);
}
} // namespace common
