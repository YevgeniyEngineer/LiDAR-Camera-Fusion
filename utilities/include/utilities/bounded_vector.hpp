#pragma once

#include <algorithm>   // std::copy
#include <cstddef>     // std::ptrdiff_t
#include <cstdint>     // std::size_t
#include <iterator>    // std::random_access_iterator
#include <memory>      // std::allocator
#include <stdexcept>   // std::runtime_error, std::out_of_range
#include <type_traits> // std::is_same_v
#include <utility>     // std::forward, std::move, std::swap

namespace utilities
{
template <typename T, std::size_t MaxSize> class BoundedVector final
{
    static_assert(MaxSize > 0U, "BoundedVector cannot have 0 max size.");

  public:
    static constexpr std::size_t MAX_SIZE = MaxSize;

    using size_type = std::size_t;
    using value_type = T;
    using pointer = T *;
    using reference = T &;

    /// @brief Swap function.
    friend void swap(BoundedVector &first, BoundedVector &second) noexcept
    {
        using std::swap;
        swap(first.size_, second.size_);
        swap(first.data_, second.data_);
    }

    /// @brief Default constructor.
    /// Preallocates MAX_SIZE memory for the container.
    BoundedVector() : size_{0U}, data_{allocator_.allocate(MAX_SIZE)}
    {
    }

    /// @brief Constructs container and resizes to MAX_SIZE
    BoundedVector(const size_type size) : size_{size}, data_{allocator_.allocate(MAX_SIZE)}
    {
        // Check the size
        if (size_ > MAX_SIZE)
        {
            throw std::runtime_error("Cannot allocate memory beyond maximum size.");
        }

        // Default construct elements
        for (size_type i = 0U; i < size_; ++i)
        {
            allocator_.construct(data_ + i, T{});
        }
    }

    /// @brief Copy constructor.
    BoundedVector(const BoundedVector &other) : size_{other.size_}, data_{allocator_.allocate(MAX_SIZE)}
    {
        // Copy only the required contents
        for (size_type i = 0U; i < other.size_; ++i)
        {
            allocator_.construct(data_ + i, other.data_[i]);
        }
    }

    /// @brief Copy assignment operator.
    BoundedVector &operator=(const BoundedVector &other)
    {
        if (this != &other)
        {
            BoundedVector temp{other}; // Use copy constructor
            swap(*this, temp);
        }
        return *this;
    }

    /// @brief Move constructor.
    BoundedVector(BoundedVector &&other) noexcept
        : size_{other.size_}, data_{other.data_}, allocator_{std::move(other.allocator_)}
    {
        other.size_ = 0U;
        other.data_ = nullptr;
    }

    /// @brief Move assignment operator.
    BoundedVector &operator=(BoundedVector &&other) noexcept
    {
        if (this != &other)
        {
            clear();                                // Destroy existing elements
            allocator_.deallocate(data_, MAX_SIZE); // Deallocates existing memory

            size_ = other.size_;
            data_ = other.data_;
            allocator_ = std::move(other.allocator_); // Move the allocator

            other.size_ = 0U;
            other.data_ = nullptr;
        }

        return *this;
    }

    /// @brief Destructor.
    ~BoundedVector() noexcept
    {
        clear();                                // Destroy constructed objects
        allocator_.deallocate(data_, MAX_SIZE); // Deallocate memory
    }

    /// @brief Mutable forward iterator.
    class iterator final
    {
      public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T *;
        using reference = T &;

        explicit iterator(pointer ptr) noexcept : ptr_{ptr}
        {
        }

        inline reference operator*() const noexcept
        {
            return *ptr_;
        }
        inline pointer operator->() noexcept
        {
            return ptr_;
        }

        // Arithmetic operations
        inline iterator &operator++() noexcept
        {
            ptr_++;
            return *this;
        }
        inline iterator operator++(int) noexcept
        {
            iterator temp = *this;
            ++ptr_;
            return temp;
        }
        inline iterator &operator--() noexcept
        {
            ptr_--;
            return *this;
        }
        inline iterator operator--(int) noexcept
        {
            iterator temp = *this;
            --ptr_;
            return temp;
        }
        inline iterator &operator+=(const difference_type n) noexcept
        {
            ptr_ += n;
            return *this;
        }
        inline iterator &operator-=(const difference_type n) noexcept
        {
            ptr_ -= n;
            return *this;
        }
        inline iterator operator+(const difference_type n) const noexcept
        {
            return iterator{ptr_ + n};
        }
        inline iterator operator-(const difference_type n) const noexcept
        {
            return iterator{ptr_ - n};
        }
        inline difference_type operator-(const iterator &other) const noexcept
        {
            return (ptr_ - other.ptr_);
        }

        // Comparison operators
        inline friend bool operator==(const iterator &a, const iterator &b) noexcept
        {
            return (a.ptr_ == b.ptr_);
        }
        inline friend bool operator!=(const iterator &a, const iterator &b) noexcept
        {
            return (a.ptr_ != b.ptr_);
        }
        inline friend bool operator<(const iterator &a, const iterator &b) noexcept
        {
            return (a.ptr_ < b.ptr_);
        }
        inline friend bool operator>(const iterator &a, const iterator &b) noexcept
        {
            return (a.ptr_ > b.ptr_);
        }
        inline friend bool operator<=(const iterator &a, const iterator &b) noexcept
        {
            return (a.ptr_ <= b.ptr_);
        }
        inline friend bool operator>=(const iterator &a, const iterator &b) noexcept
        {
            return (a.ptr_ >= b.ptr_);
        }

        // Subscript operator
        inline reference operator[](const difference_type n) const noexcept
        {
            return *(ptr_ + n);
        }

      private:
        pointer ptr_;
    };

    /// @brief Immutable forward iterator.
    class const_iterator final
    {
      public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = const T;
        using difference_type = std::ptrdiff_t;
        using pointer = const T *;
        using reference = const T &;

        explicit const_iterator(pointer ptr) noexcept : ptr_{ptr}
        {
        }

        inline reference operator*() const noexcept
        {
            return *ptr_;
        }
        inline pointer operator->() const noexcept
        {
            return ptr_;
        }

        // Arithmetic operations
        inline const_iterator &operator++() noexcept
        {
            ptr_++;
            return *this;
        }
        inline const_iterator operator++(int) noexcept
        {
            const_iterator temp = *this;
            ++ptr_;
            return temp;
        }
        inline const_iterator &operator--() noexcept
        {
            ptr_--;
            return *this;
        }
        inline const_iterator operator--(int) noexcept
        {
            const_iterator temp = *this;
            --ptr_;
            return temp;
        }
        inline const_iterator &operator+=(const difference_type n) noexcept
        {
            ptr_ += n;
            return *this;
        }
        inline const_iterator &operator-=(const difference_type n) noexcept
        {
            ptr_ -= n;
            return *this;
        }
        inline const_iterator operator+(const difference_type n) const noexcept
        {
            return const_iterator{ptr_ + n};
        }
        inline const_iterator operator-(const difference_type n) const noexcept
        {
            return const_iterator{ptr_ - n};
        }
        inline difference_type operator-(const const_iterator &other) const noexcept
        {
            return (ptr_ - other.ptr_);
        }

        // Comparison operators
        inline friend bool operator==(const const_iterator &a, const const_iterator &b) noexcept
        {
            return (a.ptr_ == b.ptr_);
        }
        inline friend bool operator!=(const const_iterator &a, const const_iterator &b) noexcept
        {
            return (a.ptr_ != b.ptr_);
        }
        inline friend bool operator<(const const_iterator &a, const const_iterator &b) noexcept
        {
            return (a.ptr_ < b.ptr_);
        }
        inline friend bool operator>(const const_iterator &a, const const_iterator &b) noexcept
        {
            return (a.ptr_ > b.ptr_);
        }
        inline friend bool operator<=(const const_iterator &a, const const_iterator &b) noexcept
        {
            return (a.ptr_ <= b.ptr_);
        }
        inline friend bool operator>=(const const_iterator &a, const const_iterator &b) noexcept
        {
            return (a.ptr_ >= b.ptr_);
        }

        // Subscript operator
        inline reference operator[](const difference_type n) const noexcept
        {
            return *(ptr_ + n);
        }

      private:
        pointer ptr_;
    };

    /// @brief Mutable reverse iterator.
    class reverse_iterator final
    {
      public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T *;
        using reference = T &;

        explicit reverse_iterator(pointer ptr) noexcept : ptr_{ptr}
        {
        }

        // Dereference operators
        inline reference operator*() const noexcept
        {
            return *ptr_;
        }
        inline pointer operator->() noexcept
        {
            return ptr_;
        }

        // Arithmetic operations
        // Note that increment and decrement operations are reversed
        inline reverse_iterator &operator++() noexcept
        {
            ptr_--;
            return *this;
        }
        inline reverse_iterator operator++(int) noexcept
        {
            reverse_iterator temp = *this;
            --ptr_;
            return temp;
        }
        inline reverse_iterator &operator--() noexcept
        {
            ptr_++;
            return *this;
        }
        inline reverse_iterator operator--(int) noexcept
        {
            reverse_iterator temp = *this;
            ++ptr_;
            return temp;
        }
        inline reverse_iterator &operator+=(const difference_type n) noexcept
        {
            ptr_ -= n;
            return *this;
        }
        inline reverse_iterator &operator-=(const difference_type n) noexcept
        {
            ptr_ += n;
            return *this;
        }
        inline reverse_iterator operator+(const difference_type n) const noexcept
        {
            return reverse_iterator{ptr_ - n};
        }
        inline reverse_iterator operator-(const difference_type n) const noexcept
        {
            return reverse_iterator{ptr_ + n};
        }
        inline difference_type operator-(const reverse_iterator &other) const noexcept
        {
            return (other.ptr_ - ptr_);
        }

        // Comparison operators
        inline friend bool operator==(const reverse_iterator &a, const reverse_iterator &b) noexcept
        {
            return (a.ptr_ == b.ptr_);
        }
        inline friend bool operator!=(const reverse_iterator &a, const reverse_iterator &b) noexcept
        {
            return (a.ptr_ != b.ptr_);
        }
        inline friend bool operator<(const reverse_iterator &a, const reverse_iterator &b) noexcept
        {
            return (a.ptr_ > b.ptr_);
        }
        inline friend bool operator>(const reverse_iterator &a, const reverse_iterator &b) noexcept
        {
            return (a.ptr_ < b.ptr_);
        }
        inline friend bool operator<=(const reverse_iterator &a, const reverse_iterator &b) noexcept
        {
            return (a.ptr_ >= b.ptr_);
        }
        inline friend bool operator>=(const reverse_iterator &a, const reverse_iterator &b) noexcept
        {
            return (a.ptr_ <= b.ptr_);
        }

        // Subscript operator
        inline reference operator[](const difference_type n) const noexcept
        {
            return *(ptr_ - n);
        }

      private:
        pointer ptr_;
    };

    /// @brief Immutable reverse iterator.
    class const_reverse_iterator final
    {
      public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = const T;
        using difference_type = std::ptrdiff_t;
        using pointer = const T *;
        using reference = const T &;

        explicit const_reverse_iterator(pointer ptr) noexcept : ptr_{ptr}
        {
        }

        // Dereference operators
        inline reference operator*() const noexcept
        {
            return *ptr_;
        }
        inline pointer operator->() const noexcept
        {
            return ptr_;
        }

        // Arithmetic operations
        // Increment and decrement operations are reversed
        inline const_reverse_iterator &operator++() noexcept
        {
            ptr_--;
            return *this;
        }
        inline const_reverse_iterator operator++(int) noexcept
        {
            const_reverse_iterator temp = *this;
            --ptr_;
            return temp;
        }
        inline const_reverse_iterator &operator--() noexcept
        {
            ptr_++;
            return *this;
        }
        inline const_reverse_iterator operator--(int) noexcept
        {
            const_reverse_iterator temp = *this;
            ++ptr_;
            return temp;
        }
        inline const_reverse_iterator &operator+=(const difference_type n) noexcept
        {
            ptr_ -= n;
            return *this;
        }
        inline const_reverse_iterator &operator-=(const difference_type n) noexcept
        {
            ptr_ += n;
            return *this;
        }
        inline const_reverse_iterator operator+(const difference_type n) const noexcept
        {
            return const_reverse_iterator{ptr_ - n};
        }
        inline const_reverse_iterator operator-(const difference_type n) const noexcept
        {
            return const_reverse_iterator{ptr_ + n};
        }
        inline difference_type operator-(const const_reverse_iterator &other) const noexcept
        {
            return (other.ptr_ - ptr_);
        }

        // Comparison operators
        inline friend bool operator==(const const_reverse_iterator &a, const const_reverse_iterator &b) noexcept
        {
            return (a.ptr_ == b.ptr_);
        }
        inline friend bool operator!=(const const_reverse_iterator &a, const const_reverse_iterator &b) noexcept
        {
            return (a.ptr_ != b.ptr_);
        }
        inline friend bool operator<(const const_reverse_iterator &a, const const_reverse_iterator &b) noexcept
        {
            return (a.ptr_ > b.ptr_);
        }
        inline friend bool operator>(const const_reverse_iterator &a, const const_reverse_iterator &b) noexcept
        {
            return (a.ptr_ < b.ptr_);
        }
        inline friend bool operator<=(const const_reverse_iterator &a, const const_reverse_iterator &b) noexcept
        {
            return (a.ptr_ >= b.ptr_);
        }
        inline friend bool operator>=(const const_reverse_iterator &a, const const_reverse_iterator &b) noexcept
        {
            return (a.ptr_ <= b.ptr_);
        }

        // Subscript operator
        inline reference operator[](const difference_type n) const noexcept
        {
            return *(ptr_ - n);
        }

      private:
        pointer ptr_;
    };

    inline iterator begin() noexcept
    {
        return iterator{data_};
    }

    inline iterator end() noexcept
    {
        return iterator{data_ + static_cast<std::ptrdiff_t>(size_)};
    }

    inline const_iterator cbegin() const noexcept
    {
        return const_iterator{data_};
    }

    inline const_iterator cend() const noexcept
    {
        return const_iterator{data_ + static_cast<std::ptrdiff_t>(size_)};
    }

    inline reverse_iterator rbegin() noexcept
    {
        return reverse_iterator{data_ + static_cast<std::ptrdiff_t>(size_) - 1};
    }

    inline reverse_iterator rend() noexcept
    {
        return reverse_iterator{data_ - 1};
    }

    inline const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator{data_ + static_cast<std::ptrdiff_t>(size_) - 1};
    }

    inline const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator{data_ - 1};
    }

    /// @brief Returns maximum bounded size of the vector.
    static constexpr inline size_type max_size() noexcept
    {
        return MAX_SIZE;
    }

    /// @brief Returns current size of the vector.
    inline size_type size() const noexcept
    {
        return size_;
    }

    /// @brief Checks if the container is empty.
    inline bool empty() const noexcept
    {
        return (size_ == 0U);
    }

    /// @brief Returns the first element.
    /// @throws std::runtime_error if empty.
    inline T &front()
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("BoundedVector is empty.");
        }

        return data_[0U];
    }

    /// @brief Returns the first element.
    /// @throws std::runtime_error if empty.
    inline const T &front() const
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("BoundedVector is empty.");
        }

        return data_[0U];
    }

    /// @brief Returns the last element.
    /// @throws std::runtime_error if empty.
    inline T &back()
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("BoundedVector is empty.");
        }

        return data_[size_ - 1U];
    }

    /// @brief Returns the last element.
    /// @throws std::runtime_error if empty.
    inline const T &back() const
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("BoundedVector is empty.");
        }

        return data_[size_ - 1U];
    }

    /// @brief Adds a value to the end of the container.
    /// @throws std::runtime_error if exceeded maximum size.
    template <typename U> inline void push_back(U &&value)
    {
        static_assert(std::is_same_v<U, T>, "<U> Value must match <T>.");

        if (size_ >= MAX_SIZE)
        {
            throw std::runtime_error("Exceeded capacity");
        }

        allocator_.construct(data_ + size_, std::forward<U>(value));
        ++size_;
    }

    /// @brief Constructs a value from input arguments at the end of the container.
    /// @throws std::runtime_error if exceeded maximum size.
    template <typename... Args> inline void emplace_back(Args &&...args)
    {
        static_assert(std::is_constructible_v<T, Args...>, "<T> must be constructible from Args.");

        if (size_ >= MAX_SIZE)
        {
            throw std::runtime_error("Capacity exceeded.");
        }

        allocator_.construct(data_ + size_, std::forward<Args>(args)...);
        ++size_;
    }

    /// @brief Does not change contents of the container.
    /// @throws std::runtime_error If trying to pop from an empty container.
    inline void pop_back()
    {
        if (size_ == 0U)
        {
            throw std::runtime_error("Attempting to remove an element from the empty container.");
        }

        --size_;
        allocator_.destroy(data_ + size_); // Destroy object
    }

    /// @brief Does not modify the underlying storage.
    /// Changes the size of the container to 0.
    inline void clear() noexcept
    {
        while (size_ > 0U)
        {
            --size_;
            allocator_.destroy(data_ + size_); // Destroy object
        }
    }

    /// @brief Returns value at a specified index.
    /// @throws std::out_of_range If index is exceeded.
    inline T &at(const size_type index)
    {
        if (index >= size_)
        {
            throw std::out_of_range("Index out of range.");
        }

        return data_[index];
    }

    /// @brief Returns value at a specified index.
    /// @throws std::out_of_range If index is exceeded.
    inline const T &at(const size_type index) const
    {
        if (index >= size_)
        {
            throw std::out_of_range("Index out of range.");
        }

        return data_[index];
    }

    /// @brief Returns value at a specified index.
    inline T &operator[](const size_type index) noexcept
    {
        return data_[index];
    }

    /// @brief Returns value at a specified index.
    inline const T &operator[](const size_type index) const noexcept
    {
        return data_[index];
    }

    /// @brief Returns pointer to the raw memory.
    inline const T *data() const
    {
        return data_;
    }

  private:
    size_type size_;
    std::allocator<T> allocator_;
    T *data_; // Raw pointer to the allocated memory
};

} // namespace utilities
