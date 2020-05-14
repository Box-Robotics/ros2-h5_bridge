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
#ifndef H5B_SENSOR_MSGS__PCL_H_
#define H5B_SENSOR_MSGS__PCL_H_

#include <string>
#include <h5_bridge.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace h5b_sensor_msgs
{
  /**
   * Writes a `sensor_msgs::msg::PointCloud2` to an HDF5 group `group` in the
   * file managed by `h5`.
   *
   * The point cloud is serialized to a group with datasets underneath that map
   * one dataset for each of its fields. The exception is, if the point cloud
   * has fields `x`, `y`, and `z`, those data are bundled together into one
   * 3-channel array. We consider this a form of the "struct of arrays"
   * approach to serialization versus the "array of structs" approach (that a
   * typical point cloud employs).
   *
   * NOTE: no notion of the PCL point type is known. As such, any padding is
   * not possible to reconstruct.
   *
   * @param[in] h5 Pointer to an open `H5File`.
   * @param[in] group The fullpath name of the group to write to
   * @param[in] pcl The sensor_msgs::msg::PointCloud2 to serialize
   * @param[in] gzip The gzip compression level
   */
  void write(h5_bridge::H5File * h5, const std::string& group,
             const sensor_msgs::msg::PointCloud2& pcl, int gzip = 0);

  /**
   * Reads the group `group` from the HDF5 file pointed to by `h5` and returns
   * a `sensor_msgs::msg::PointCloud2` representation of child datasets of that
   * group.
   *
   * This function assumes the group is annotated in the way at `write`
   * annotates it. This provides the deserializer with the meta-info necessary
   * to properly reconstruct the data. NOTE: no notion of the PCL point type is
   * known. As such, any padding is not possible to reconstruct.
   *
   * @param[in] h5 Pointer to an open `H5File`.
   * @param[in] group The fullpath name of the group to read from
   *
   * @return A `sensor_msgs::msg::PointCloud2` representation of the H5
   *         child datasets of the group
   */
  sensor_msgs::msg::PointCloud2
  toPointCloud2Msg(h5_bridge::H5File * h5, const std::string& group);

} // end: namespace h5b_sensor_msgs

#endif // H5B_SENSOR_MSGS__PCL_H_
