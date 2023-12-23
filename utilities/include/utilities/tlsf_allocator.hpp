#ifndef UTILITIES__TLSF_HPP
#define UTILITIES__TLSF_HPP

extern "C"
{
#include "tlsf/tlsf.h"
}

#include <cstddef>     // std::ptrdiff_t
#include <cstdlib>     // std::malloc
#include <cstring>     // std::memset
#include <memory>      // std::address_of
#include <stdexcept>   // std::bad_alloc
#include <type_traits> // std::true_type

namespace utilities
{
template <typename T> class TLSFAllocator
{
  public:
    using value_type = T;
    using pointer = T *;
    using const_pointer = const T *;
    using reference = T &;
    using const_reference = const T &;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using propagate_on_container_move_assignment = std::true_type;
    using is_always_equal = std::true_type;

    template <typename U> struct rebind
    {
        using other = TLSFAllocator<U>;
    };

    TLSFAllocator() : tlsf_instance_{}
    {
    }

    TLSFAllocator(const TLSFAllocator &other) : tlsf_instance_(other.tlsf_instance_)
    {
    }

    template <typename U> TLSFAllocator(const TLSFAllocator<U> &other) : tlsf_instance_(other.tlsf_instance_)
    {
    }

    ~TLSFAllocator()
    {
    }

    [[nodiscard]] inline pointer address(reference x) const noexcept
    {
        return std::addressof(x);
    }

    [[nodiscard]] inline const_pointer address(const_reference x) const noexcept
    {
        return std::addressof(x);
    }

    [[nodiscard]] pointer allocate(size_type n)
    {
        if (n > 0)
        {
            void *memory = tlsf_malloc(&tlsf_instance_, n * sizeof(T));

            if (nullptr == memory)
            {
                // Could not allocate memory
                throw std::bad_alloc();
            }

            return static_cast<pointer>(memory);
        }
        else
        {
            return nullptr;
        }
    }

    void deallocate(pointer p, size_type n)
    {
        tlsf_free(&tlsf_instance_, p);
    }

    template <typename U, typename... Args> inline void construct(U *p, Args &&...args)
    {
        ::new (static_cast<void *>(p)) U(std::forward<Args>(args)...);
    }

    inline void destroy(pointer p)
    {
        p->~T();
    }

    template <typename U> inline void destroy(U *p)
    {
        p->~U();
    }

    [[nodiscard]] inline bool operator==(const TLSFAllocator &other) const noexcept
    {
        // assuming all instances of this allocator are interchangeable.
        return true;
    }

    [[nodiscard]] inline bool operator!=(const TLSFAllocator &other) const
    {
        // assuming all instances of this allocator are interchangeable.
        return false;
    }

  private:
    tlsf_t tlsf_instance_;
};
} // namespace utilities

#endif // UTILITIES__TLSF_HPP
