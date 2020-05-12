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

  /**
   * Runtime endian check
   *
   * @return True if the current machine is little endian
   */
  inline bool little_endian()
  {
    std::uint16_t dummy = 0x1;
    std::uint8_t *dummy_ptr = reinterpret_cast<std::uint8_t*>(&dummy);
    return dummy_ptr[0] == 0x1 ? true : false;
  }

  /**
   * Runtime endian check
   *
   * @return True if the current machine is big endian
   */
  inline bool big_endian()
  {
    return ! h5_bridge::little_endian();
  }

} // end: namespace h5_bridge

#endif // H5_BRIDGE__ERR_H_
