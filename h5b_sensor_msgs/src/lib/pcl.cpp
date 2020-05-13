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
#include <h5b_sensor_msgs/pcl.hpp>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <h5_bridge.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

bool
has_xyz(const sensor_msgs::msg::PointCloud2& msg)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  std::uint8_t x_t = 0;
  std::uint8_t y_t = 0;
  std::uint8_t z_t = 0;

  for (auto& field : msg.fields)
    {
      if (field.name == "x")
        {
          has_x = true;
          x_t = field.datatype;
        }
      else if (field.name == "y")
        {
          has_y = true;
          y_t = field.datatype;
        }
      else if (field.name == "z")
        {
          has_z = true;
          z_t = field.datatype;
        }
    }

  bool retval = has_x && has_y && has_z;

  if (retval)
    {
      if (!((x_t == y_t) && (x_t == z_t)))
        {
          retval = false;
        }
    }

  return retval;
}

h5_bridge::DSet_t
point_field_datatype_to_dset_t(std::uint8_t pf_dt)
{
  h5_bridge::DSet_t dset_dt;

  switch (pf_dt)
    {
    case sensor_msgs::msg::PointField::INT8:
      dset_dt = std::int8_t{0};
      break;

    case sensor_msgs::msg::PointField::UINT8:
      dset_dt = std::uint8_t{0};
      break;

    case sensor_msgs::msg::PointField::INT16:
      dset_dt = std::int16_t{0};
      break;

    case sensor_msgs::msg::PointField::UINT16:
      dset_dt = std::uint16_t{0};
      break;

    case sensor_msgs::msg::PointField::INT32:
      dset_dt = std::int32_t{0};
      break;

    case sensor_msgs::msg::PointField::UINT32:
      dset_dt = std::uint32_t{0};
      break;

    case sensor_msgs::msg::PointField::FLOAT32:
      dset_dt = float{0};
      break;

    case sensor_msgs::msg::PointField::FLOAT64:
      dset_dt = double{0};
      break;
    }

  return dset_dt;
}

void
h5b_sensor_msgs::write(h5_bridge::H5File * h5, const std::string& group,
                       const sensor_msgs::msg::PointCloud2& pcl, int gzip)
{
  bool xyz_ok = has_xyz(pcl);

  if (xyz_ok)
    {
      std::uint8_t xyz_tp = 0;
      for (auto& field : pcl.fields)
        {
          if ((field.name == "x") ||
              (field.name == "y") ||
              (field.name == "z"))
            {
              xyz_tp = field.datatype;
              break;
            }
        }

      h5_bridge::DSet_t xyz_dt = point_field_datatype_to_dset_t(xyz_tp);
      std::visit([&](const auto& dt) -> void
                 {
                   auto tp = decltype(dt){0};

                   std::vector<decltype(tp)> xyz(pcl.width*pcl.height*3);
                   std::uint32_t vec_idx = 0;
                   for (sensor_msgs::PointCloud2ConstIterator<decltype(tp)>
                          iter_x(pcl, "x"), iter_y(pcl, "y"), iter_z(pcl, "z");
                        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
                     {
                       xyz[vec_idx] = *iter_x;
                       xyz[vec_idx + 1] = *iter_y;
                       xyz[vec_idx + 2] = *iter_z;

                       vec_idx += 3;
                     }

                   h5->write(
                     group + "/xyz", xyz, pcl.height, pcl.width, 3, gzip);
                 }, xyz_dt);
    }

  for (auto& field : pcl.fields)
    {
      if (((field.name == "x") ||
           (field.name == "y") ||
           (field.name == "z")) && xyz_ok)
        {
          continue;
        }

      h5_bridge::DSet_t field_dt =
        point_field_datatype_to_dset_t(field.datatype);
      std::visit([&](const auto& dt) -> void
                 {
                   auto tp = decltype(dt){0};

                   std::vector<decltype(tp)> vec(pcl.width*pcl.height);
                   std::uint32_t vec_idx = 0;
                   for (sensor_msgs::PointCloud2ConstIterator<decltype(tp)>
                          iter(pcl, field.name); iter != iter.end(); ++iter)
                     {
                       vec[vec_idx] = *iter;
                       vec_idx += 1;
                     }

                   h5->write(group + "/" + field.name,
                             vec, pcl.height, pcl.width, 1, gzip);
                 }, field_dt);
    }

  auto grp = h5->group(group);
  h5->set_attr(grp, "header.frame_id", pcl.header.frame_id);
  h5->set_attr(grp, "header.stamp.sec", pcl.header.stamp.sec);
  h5->set_attr(grp, "header.stamp.nanosec", pcl.header.stamp.nanosec);
  h5->set_attr(grp, "is_bigendian", pcl.is_bigendian);
  h5->set_attr(grp, "is_dense", pcl.is_dense);
}
