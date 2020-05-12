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
#ifndef H5B_SENSOR_MSGS__IMAGE_H_
#define H5B_SENSOR_MSGS__IMAGE_H_

#include <string>
#include <h5_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace h5b
{
  /**
   * Writes a `sensor_msgs::msg::Image` to an HDF5 dataset `dset` in the file
   * managed by `h5`.
   *
   * @param[in] h5 Pointer to an open `H5File`.
   * @param[in] dset The fullpath name of the dataset to write to
   * @param[in] img The sensor_msgs::msg::Image to serialize
   * @param[in] gzip The gzip compression level
   */
  void write(h5_bridge::H5File * h5, const std::string& dset,
             const sensor_msgs::msg::Image& img, int gzip = 0);

  sensor_msgs::msg::Image
  toImageMsg(h5_bridge::H5File * h5, const std::string& dset);

  template<typename T>
  T read(h5_bridge::H5File * h5, const std::string& dset)
  {
    return h5b::toImageMsg(h5, dset);
  }

} // end: namespace h5b

#endif // H5B_SENSOR_MSGS__IMAGE_H_
