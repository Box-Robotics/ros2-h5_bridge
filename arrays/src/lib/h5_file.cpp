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
#include <arrays/h5_bridge/h5_file.hpp>
#include <filesystem>
#include <optional>
#include <string>
#include <arrays/err.hpp>
#include <impl/h5_file_impl.hpp>

namespace fs = std::filesystem;

arrays::h5_bridge::H5File::H5File(const std::string& path,
                                  const std::string& mode)
  : pImpl(new arrays::h5_bridge::H5File::Impl(path, mode))
{}

arrays::h5_bridge::H5File::H5File(const fs::path& path,
                                  const std::string& mode)
  : pImpl(new arrays::h5_bridge::H5File::Impl(path.native(), mode))
{}

arrays::h5_bridge::H5File::H5File(arrays::h5_bridge::H5File&& obj) = default;

arrays::h5_bridge::H5File&
arrays::h5_bridge::H5File::operator=(arrays::h5_bridge::H5File&& obj) = default;

arrays::h5_bridge::H5File::~H5File() = default;

std::string
arrays::h5_bridge::H5File::filename()
{
  return this->pImpl->filename();
}

arrays::h5_bridge::H5ObjId
arrays::h5_bridge::H5File::group(const std::string& path)
{
  try
    {
      if (this->pImpl->group(path))
        {
          return path;
        }
    }
  catch (const arrays::error_t& ex)
    {
      return std::nullopt;
    }

  return std::nullopt;
}

std::vector<std::string>
arrays::h5_bridge::H5File::subgroups(const arrays::h5_bridge::H5ObjId& group)
{
  return this->pImpl->subgroups(group.value_or("/"));
}

std::vector<std::string>
arrays::h5_bridge::H5File::attributes(const arrays::h5_bridge::H5ObjId& obj)
{
  return this->pImpl->attributes(obj.value_or("/"));
}

void
arrays::h5_bridge::H5File::set_attr(
  const arrays::h5_bridge::H5ObjId& obj,
  const std::string& key,
  const arrays::h5_bridge::Attr_t& value)
{
  if (obj.has_value())
    {
      this->pImpl->set_attr(obj.value(), key, value);
    }
}

void
arrays::h5_bridge::H5File::set_attr(
  const arrays::h5_bridge::H5ObjId& obj,
  const std::string& key,
  const char * value)
{
  this->set_attr(obj, key, std::string(value));
}

void
arrays::h5_bridge::H5File::attr(
  const arrays::h5_bridge::H5ObjId& obj,
  const std::string& key,
  arrays::h5_bridge::Attr_t& value_out)
{
  this->pImpl->attr(obj.value(), key, value_out);
}

void
arrays::h5_bridge::H5File::flush()
{
  this->pImpl->flush();
}
