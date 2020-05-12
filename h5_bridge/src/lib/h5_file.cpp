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
#include <h5_bridge/h5_file.hpp>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
#include <h5_bridge/err.hpp>
#include <h5_bridge/h5_dset_t.hpp>
#include <impl/h5_file_impl.hpp>

namespace fs = std::filesystem;

h5_bridge::H5File::H5File(const std::string& path,
                          const std::string& mode)
  : pImpl(new h5_bridge::H5File::Impl(path, mode))
{}

h5_bridge::H5File::H5File(const fs::path& path,
                          const std::string& mode)
  : pImpl(new h5_bridge::H5File::Impl(path.native(), mode))
{}

h5_bridge::H5File::H5File(h5_bridge::H5File&& obj) = default;

h5_bridge::H5File&
h5_bridge::H5File::operator=(h5_bridge::H5File&& obj) = default;

h5_bridge::H5File::~H5File() = default;

std::string
h5_bridge::H5File::filename()
{
  return this->pImpl->filename();
}

h5_bridge::H5ObjId
h5_bridge::H5File::group(const std::string& path)
{
  try
    {
      if (this->pImpl->group(path))
        {
          return path;
        }
    }
  catch (const h5_bridge::error_t& ex)
    {
      return std::nullopt;
    }

  return std::nullopt;
}

h5_bridge::H5ObjId
h5_bridge::H5File::dset(const std::string& path)
{
  try
    {
      if (this->pImpl->dset(path))
        {
          return path;
        }
    }
  catch (const h5_bridge::error_t& ex)
    {
      return std::nullopt;
    }

  return std::nullopt;
}

std::vector<std::string>
h5_bridge::H5File::subgroups(const h5_bridge::H5ObjId& group)
{
  return this->pImpl->subgroups(group.value_or("/"));
}

std::vector<std::string>
h5_bridge::H5File::attributes(const h5_bridge::H5ObjId& obj)
{
  return this->pImpl->attributes(obj.value_or("/"));
}

std::vector<std::string>
h5_bridge::H5File::datasets(const h5_bridge::H5ObjId& group)
{
  return this->pImpl->datasets(group.value_or("/"));
}

void
h5_bridge::H5File::set_attr(
  const h5_bridge::H5ObjId& obj,
  const std::string& key,
  const h5_bridge::Attr_t& value)
{
  if (obj.has_value())
    {
      this->pImpl->set_attr(obj.value(), key, value);
    }
}

void
h5_bridge::H5File::set_attr(
  const h5_bridge::H5ObjId& obj,
  const std::string& key,
  const char * value)
{
  this->set_attr(obj, key, std::string(value));
}

void
h5_bridge::H5File::attr(
  const h5_bridge::H5ObjId& obj,
  const std::string& key,
  h5_bridge::Attr_t& value_out)
{
  this->pImpl->attr(obj.value(), key, value_out);
}

void
h5_bridge::H5File::clear(const h5_bridge::H5ObjId& obj)
{
  if (obj.has_value())
    {
      this->pImpl->clear(obj.value());
    }
  else
    {
      this->pImpl->clear_all();
    }
}

void
h5_bridge::H5File::flush()
{
  this->pImpl->flush();
}

void
h5_bridge::H5File::write(
  const h5_bridge::H5ObjId& obj,
  const std::uint8_t * buff,
  const h5_bridge::DSet_t dset_t,
  int rows, int cols, int chans, int gzip)
{
  if (obj.has_value())
    {
      this->pImpl->write(obj.value(), buff, dset_t, rows, cols, chans, gzip);
    }
}

std::tuple<h5_bridge::DSet_t, int, int, int>
h5_bridge::H5File::get_shape(const h5_bridge::H5ObjId& obj)
{
  return this->pImpl->read(obj.value());
}

void
h5_bridge::H5File::read(const h5_bridge::H5ObjId& obj,
                        std::uint8_t * buff)
{
  std::tie(std::ignore, std::ignore, std::ignore, std::ignore) =
    this->pImpl->read(obj.value(), buff);
}
