// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <string>       // memset
#include <string.h>     // memset
#include <cassert>
#include <cstdlib>

namespace vilo {
namespace aligned_mem {

  /// Check if the pointer is aligned to the specified byte granularity
  inline bool
  is_aligned8(const void* ptr)
  {
    return ((reinterpret_cast<size_t>(ptr)) & 0x7) == 0;
  }

  inline bool
  is_aligned16(const void* ptr)
  {
    return ((reinterpret_cast<size_t>(ptr)) & 0xF) == 0;
  }

  template <class T, int N=20> struct placement_delete
  {
    enum { Size = (1<<N) };

    struct Array {
      T data[Size];
    };

    static inline void destruct(T* buf)
    {
      (*(Array*)buf).~Array();
    }

    static inline void free(T* buf, size_t M)
    {
      if (M >= Size) {
        placement_delete<T,N>::free(buf+Size,M-Size);
        placement_delete<T,N>::destruct(buf);
      } else {
        placement_delete<T,N-1>::free(buf, M);
      }
    }
  };

  template <class T> struct placement_delete<T,-1>
  {
    static inline void free(T*, size_t ) {}
  };

  inline void * aligned_alloc(size_t count, size_t alignment){
    void * mem = NULL;
    assert(posix_memalign(&mem, alignment, count) == 0);
    return mem;
  }

  inline void aligned_free(void * memory) {
    free(memory);
  }

  template <class T>
  inline T * aligned_alloc(size_t count, size_t alignment){
    void * data = aligned_alloc(sizeof(T)* count, alignment);
    return new (data) T[count];
  }

  template <class T>
  inline void aligned_free(T * memory, size_t count){
    placement_delete<T>::free(memory, count);
    aligned_free(memory);
  }

  template<class T> inline void memfill(T* data, int n, const T val)
  {
    T* de = data + n;
    for(;data < de; data++)
      *data=val;
  }

  template<> inline void memfill(unsigned char* data, int n, const unsigned char val)
  {
    memset(data, val, n);
  }

  template<> inline void memfill(signed char* data, int n, const signed char val)
  {
    memset(data, val, n);
  }

  template<> inline void memfill(char* data, int n, const char val)
  {
    memset(data, val, n);
  }

  template <class T, int N>
  struct AlignedMem {
    T* mem;
    size_t count;
    AlignedMem(size_t c) : count(c) {
      mem = aligned_alloc<T>(count, N);
    }
    ~AlignedMem() {
      aligned_free<T>(mem, count);
    }
    T* data() { return mem; }
    const T* data() const { return mem; }
  };

} // namespace aligned_mem
} // namespace vikit
