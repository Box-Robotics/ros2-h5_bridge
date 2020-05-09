// -*- c++ -*-
/*
 * Copyright (C) 2020 Box Robotics, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef H5_BRIDGE__UTIL_H_
#define H5_BRIDGE__UTIL_H_

#include <algorithm>
#include <cstdint>
#include <functional>
#include <random>
#include <type_traits>
#include <vector>

namespace h5_bridge
{
  /**
   * Converts a value of type `T` to a host-ordered byte array
   *
   * @param[in] in The input value to convert to bytes
   * @param[out] out The buffer to fill with the byte encoding of `in`. We
   *                 assume it is large enough.
   */
  template<typename T>
  void to_bytes(const T in, unsigned char * out)
  {
    union
    {
      T v;
      unsigned char bytes[sizeof(T)];
    } value;

    value.v = in;

#if !defined(_WIN32) && __BYTE_ORDER == __ORDER_BIG_ENDIAN__
    std::copy(value.bytes, value.bytes + sizeof(T), out);
#else
    std::reverse_copy(value.bytes, value.bytes + sizeof(T), out);
#endif
  }

  /**
   * Converts a vector of values of type `T` to a vector of bytes representing
   * the input `in` in the hosts byte ordering.
   *
   * @param[in] in The vector of values to convert to bytes
   * @return A byte array representing the data of input vector `in`.
   */
  template<typename T>
  std::vector<std::uint8_t>
  to_bytes(const std::vector<T>& in)
  {
    std::vector<std::uint8_t> out;
    out.resize(in.size()*sizeof(T));

    int out_idx = 0;
    for (auto pixel : in)
      {
        to_bytes(pixel, out.data() + out_idx);
        out_idx += sizeof(pixel);
      }

    return out;
  }

  /**
   * Generates a random vector consisting of `n_elems` of type `T`. If `T` is a
   * floating point type, we draw values from a standard normal
   * distribution. Otherwise, we assume `T` is an integer type and we use a
   * uniform distribution.
   *
   * @return A random vector of `T`.
   */
  template<typename T>
  std::vector<T> random_vec(int n_elems)
  {
    std::random_device r;
    std::seed_seq seed{r(), r(), r(), r(), r(), r(), r(), r()};
    std::mt19937 eng(seed);
    std::vector<T> retval(n_elems);

    if constexpr(std::is_floating_point_v<T>)
      {
        std::normal_distribution<T> dist(T{0}, T{1.0});
        std::generate(retval.begin(), retval.end(), std::bind(dist, eng));
      }
    else
      {
        std::uniform_int_distribution<T> dist;
        std::generate(retval.begin(), retval.end(), std::bind(dist, eng));
      }

    return retval;
  }

} // end: namespace h5_bridge

#endif // H5_BRIDGE__ERR_H_
