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
#ifndef H5_BRIDGE__H5_BRIDGE_H5_ATTR_T_H_
#define H5_BRIDGE__H5_BRIDGE_H5_ATTR_T_H_

#include <cstdint>
#include <string>
#include <variant>

namespace h5_bridge
{
  using Attr_t = std::variant<std::uint8_t,
                              std::uint16_t,
                              std::uint32_t,
                              std::uint64_t,
                              std::int8_t,
                              std::int16_t,
                              std::int32_t,
                              std::int64_t,
                              float,
                              double,
                              bool,
                              std::string>;

} // end: namespace h5_bridge::h5_bridge

#endif // H5_BRIDGE__H5_BRIDGE_H5_ATTR_T_H_
