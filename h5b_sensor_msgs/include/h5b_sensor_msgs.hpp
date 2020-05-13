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
#ifndef H5B_SENSOR_MSGS__H5B_SENSOR_MSGS_H_
#define H5B_SENSOR_MSGS__H5B_SENSOR_MSGS_H_

#include <stdexcept>
#include <type_traits>
#include <sensor_msgs/msg/image.hpp>
#include <h5b_sensor_msgs/image.hpp>
#include <h5b_sensor_msgs/pcl.hpp>

namespace h5b_sensor_msgs
{
  template<typename T>
  T read(h5_bridge::H5File * h5, const std::string& path)
  {
    if constexpr(std::is_same_v<sensor_msgs::msg::Image, T>)
      {
        return h5b_sensor_msgs::toImageMsg(h5, path);
      }
    else
      {
        throw(std::domain_error("No serializer available for type"));
      }
  }
}

#endif // H5B_SENSOR_MSGS__H5B_SENSOR_MSGS_H_
