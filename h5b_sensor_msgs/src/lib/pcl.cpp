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
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <variant>
#include <vector>
#include <h5_bridge.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

std::vector<std::string>
split_string(const std::string& in, const char delim = ',')
{
  std::vector<std::string> out;

  std::stringstream ss(in);
  while (ss.good())
    {
      std::string substr;
      std::getline(ss, substr, delim);
      out.push_back(substr);
    }

  return out;
}

template<typename T>
std::vector<T>
split_string(const std::string& in, const char delim = ',')
{
  auto vec_of_string = split_string(in, delim);
  std::vector<T> out;
  std::transform(vec_of_string.begin(), vec_of_string.end(),
                 std::back_inserter(out),
                 [](const std::string& s) -> decltype(T{})
                 {
                   return static_cast<T>(std::atoi(s.c_str()));
                 });
  return out;
}

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
                   std::string dset_name = group + "/xyz";

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

                   h5->write(dset_name, xyz, pcl.height, pcl.width, 3, gzip);

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
                   std::string dset_name = group + "/" + field.name;

                   std::vector<decltype(tp)> vec(pcl.width*pcl.height);
                   std::uint32_t vec_idx = 0;
                   for (sensor_msgs::PointCloud2ConstIterator<decltype(tp)>
                          iter(pcl, field.name); iter != iter.end(); ++iter)
                     {
                       vec[vec_idx] = *iter;
                       vec_idx += 1;
                     }

                   h5->write(dset_name, vec, pcl.height, pcl.width, 1, gzip);

                 }, field_dt);
    }

  //
  // Attach PCL meta-data to the top-level group to aid the deserialization
  // routine.
  //
  auto grp = h5->group(group);
  int n_fields = pcl.fields.size();
  std::string field_names;
  std::string field_offsets;
  std::string field_datatypes;
  std::string field_counts;
  for (int i = 0; i < n_fields; ++i)
    {
      field_names += pcl.fields[i].name;
      field_offsets += std::to_string(pcl.fields[i].offset);
      field_datatypes += std::to_string(pcl.fields[i].datatype);
      field_counts += std::to_string(pcl.fields[i].count);

      if (i < (n_fields - 1))
        {
          field_names += ",";
          field_offsets += ",";
          field_datatypes += ",";
          field_counts += ",";
        }
    }
  h5->set_attr(grp, "fields.name", field_names);
  h5->set_attr(grp, "fields.offset", field_offsets);
  h5->set_attr(grp, "fields.datatype", field_datatypes);
  h5->set_attr(grp, "fields.count", field_counts);
  h5->set_attr(grp, "header.frame_id", pcl.header.frame_id);
  h5->set_attr(grp, "header.stamp.sec", pcl.header.stamp.sec);
  h5->set_attr(grp, "header.stamp.nanosec", pcl.header.stamp.nanosec);
  h5->set_attr(grp, "is_bigendian", pcl.is_bigendian);
  h5->set_attr(grp, "is_dense", pcl.is_dense);
  h5->set_attr(grp, "height", pcl.height);
  h5->set_attr(grp, "width", pcl.width);
  h5->set_attr(grp, "point_step", pcl.point_step);
  h5->set_attr(grp, "row_step", pcl.row_step);
}

sensor_msgs::msg::PointCloud2
h5b_sensor_msgs::toPointCloud2Msg(
  h5_bridge::H5File * h5, const std::string& group)
{
  H5B_INFO("Attempting to interpret group {} as PCL serialization", group);
  sensor_msgs::msg::PointCloud2 msg;

  auto grp = h5->group(group);
  auto field_names = split_string(h5->attr<std::string>(grp, "fields.name"));
  auto field_offsets =
    split_string<std::uint32_t>(h5->attr<std::string>(grp, "fields.offset"));
  auto field_datatypes =
    split_string<std::uint8_t>(h5->attr<std::string>(grp, "fields.datatype"));
  auto field_counts =
    split_string<std::uint32_t>(h5->attr<std::string>(grp, "fields.count"));

  int n_fields = field_names.size();
  for (int i = 0; i < n_fields; ++i)
    {
      try
        {
          sensor_msgs::msg::PointField pt;
          pt.name = field_names.at(i);
          pt.offset = field_offsets.at(i);
          pt.datatype = field_datatypes.at(i);
          pt.count = field_counts.at(i);

          msg.fields.push_back(pt);
        }
      catch (const std::out_of_range& ex)
        {
          H5B_ERROR("While reading meta data on: {} -> {}",
                    group, ex.what());
          throw(h5_bridge::error_t(h5_bridge::ERR_H5_CORRUPT_FILE));
        }
    }

  msg.header.frame_id = h5->attr<std::string>(grp, "header.frame_id");
  msg.header.stamp.sec = h5->attr<std::int32_t>(grp, "header.stamp.sec");
  msg.header.stamp.nanosec =
    h5->attr<std::uint32_t>(grp, "header.stamp.nanosec");
  msg.is_bigendian = h5->attr<bool>(grp, "is_bigendian");
  msg.is_dense = h5->attr<bool>(grp, "is_dense");
  msg.height = h5->attr<std::uint32_t>(grp, "height");
  msg.width = h5->attr<std::uint32_t>(grp, "width");
  msg.point_step = h5->attr<std::uint32_t>(grp, "point_step");
  msg.row_step = h5->attr<std::uint32_t>(grp, "row_step");
  msg.data.resize(msg.row_step*msg.height);

  bool xyz_ok = has_xyz(msg);

  if (xyz_ok)
    {
      std::uint8_t xyz_tp = 0;
      for (auto& field : msg.fields)
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
                   std::string dset_name = group + "/xyz";

                   std::vector<decltype(tp)> xyz;
                   std::tie(xyz, std::ignore, std::ignore, std::ignore) =
                     h5->read<decltype(tp)>(dset_name);

                   std::uint32_t vec_idx = 0;
                   for (sensor_msgs::PointCloud2Iterator<decltype(tp)>
                          iter_x(msg, "x"), iter_y(msg, "y"), iter_z(msg, "z");
                        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
                     {
                       *iter_x = xyz[vec_idx];
                       *iter_y = xyz[vec_idx + 1];
                       *iter_z = xyz[vec_idx + 2];

                       vec_idx += 3;
                     }
                 }, xyz_dt);
    }

  for (auto& field : msg.fields)
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
                   std::string dset_name = group + "/" + field.name;

                   std::vector<decltype(tp)> vec;
                   std::tie(vec, std::ignore, std::ignore, std::ignore) =
                     h5->read<decltype(tp)>(dset_name);

                   std::uint32_t vec_idx = 0;
                   for (sensor_msgs::PointCloud2Iterator<decltype(tp)>
                          iter(msg, field.name); iter != iter.end(); ++iter)
                     {
                       *iter = vec[vec_idx];
                       vec_idx += 1;
                     }
                 }, field_dt);
    }

  return msg;
}
