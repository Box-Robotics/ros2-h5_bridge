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
#ifndef H5_BRIDGE__H5_BRIDGE_H5_DSET_T_H_
#define H5_BRIDGE__H5_BRIDGE_H5_DSET_T_H_

#include <cstdint>
#include <variant>

namespace h5_bridge
{
  using DSet_t = std::variant<std::uint8_t,
                              std::int8_t,
                              std::uint16_t,
                              std::int16_t,
                              std::int32_t,
                              float,
                              double>;

} // end: namespace h5_bridge

#endif // H5_BRIDGE__H5_BRIDGE_H5_DSET_T_H_