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
#ifndef ARRAYS__H5_BRIDGE_H5_FILE_IMPL_H_
#define ARRAYS__H5_BRIDGE_H5_FILE_IMPL_H_

#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <H5Cpp.h>
#include <arrays/err.hpp>
#include <arrays/logging.hpp>
#include <arrays/h5_bridge/h5_attr_t.hpp>

namespace fs = std::filesystem;
using H5ObjectPtr = std::shared_ptr<H5::H5Object>;
using H5FilePtr = std::shared_ptr<H5::H5File>;
using H5GroupPtr = std::shared_ptr<H5::Group>;
using H5AttrPtr = std::shared_ptr<H5::Attribute>;
using H5DSetPtr = std::shared_ptr<H5::DataSet>;

//-----------------------------
// Data type mapping for attributes
//-----------------------------

const auto& h5_bool_ = []() -> H5::EnumType
{
  H5::EnumType retval = H5::EnumType(H5::PredType::NATIVE_INT8);

  std::int8_t f = 0;
  std::int8_t t = 1;

  retval.insert("FALSE", &f);
  retval.insert("TRUE", &t);

  return retval;
};

struct H5DTypeVisitor
{
  H5::DataType operator() (std::uint8_t) const
  { return H5::PredType::NATIVE_UINT8; }

  H5::DataType operator() (std::uint16_t) const
  { return H5::PredType::NATIVE_UINT16; }

  H5::DataType operator() (std::uint32_t) const
  { return H5::PredType::NATIVE_UINT32; }

  H5::DataType operator() (std::uint64_t) const
  { return H5::PredType::NATIVE_UINT64; }

  H5::DataType operator() (std::int8_t) const
  { return H5::PredType::NATIVE_INT8; }

  H5::DataType operator() (std::int16_t) const
  { return H5::PredType::NATIVE_INT16; }

  H5::DataType operator() (std::int32_t) const
  { return H5::PredType::NATIVE_INT32; }

  H5::DataType operator() (std::int64_t) const
  { return H5::PredType::NATIVE_INT64; }

  H5::DataType operator() (float) const
  { return H5::PredType::NATIVE_FLOAT; }

  H5::DataType operator() (double) const
  { return H5::PredType::NATIVE_DOUBLE; }

  H5::DataType operator() (bool) const
  { return h5_bool_(); }

  H5::DataType operator() (const std::string&) const
  { return H5::StrType(H5::PredType::C_S1, H5T_VARIABLE); }
};

const static int ARRAYS_H5_VERBOSE =
  std::getenv("ARRAYS_H5_VERBOSE") == nullptr ? 0 : 1;

//-----------------------------
// Impl interface
//-----------------------------

namespace arrays::h5_bridge
{
  class H5File::Impl
  {
  public:
    Impl(const std::string& path, const std::string& mode);
    ~Impl() = default;

    std::string filename();
    bool group(const std::string& path, bool create = true);
    std::vector<std::string> subgroups(const std::string& path);
    std::vector<std::string> attributes(const std::string& path);
    void set_attr(const std::string& path, const std::string& key,
                  const arrays::h5_bridge::Attr_t& value);
    void attr(const std::string& path, const std::string& key,
              arrays::h5_bridge::Attr_t& attr_out);
    void flush();

  protected:
    std::string name(hid_t id);
    // pass `tp = H5I_BADID` to ignore the type check and just
    // see if the `path` is cached.
    bool is_cached(const std::string& path, const H5I_type_t tp);
    std::optional<H5AttrPtr> attr_obj(
      H5::H5Object *obj, const std::string& name);

    std::unordered_map<std::string, H5ObjectPtr> obj_cache_;
    H5FilePtr h5_;
    std::string fname_;
  };

} // end: namespace arrays::h5_bridge

//-----------------------------
// Impl implementation
//-----------------------------

arrays::h5_bridge::H5File::Impl::Impl(
 const std::string& path, const std::string& mode)
  : fname_(path)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  try
    {
      if (mode == "r")
        {
          // read-only, file must exist
          this->h5_ = std::make_shared<H5::H5File>(path, H5F_ACC_RDONLY);
        }
      else if (mode == "r+")
        {
          // read-write, file must exist
          this->h5_ = std::make_shared<H5::H5File>(path, H5F_ACC_RDWR);
        }
      else if (mode == "w")
        {
          // create file, truncate if exists
          this->h5_ = std::make_shared<H5::H5File>(path, H5F_ACC_TRUNC);
        }
      else if ((mode == "w-") || (mode == "x"))
        {
          // create file, fail if exists
          this->h5_ = std::make_shared<H5::H5File>(path, H5F_ACC_EXCL);
        }
      else if (mode == "a")
        {
          // read-write if exists, create otherwise
          this->h5_ = std::make_shared<H5::H5File>(
            path, fs::exists(fs::path(path)) ? H5F_ACC_RDWR : H5F_ACC_EXCL);
        }
      else
        {
          ALOG_ERROR("Unrecoginized H5 open mode: {}", mode);
          throw(arrays::error_t(arrays::ERR_H5_BAD_MODE));
        }
    }
  catch (const H5::Exception& ex)
    {
      ALOG_ERROR("Failed to open {} with mode {}: {}",
                 path, mode, ex.getDetailMsg());
      throw(arrays::error_t(arrays::ERR_H5_OPEN_FAILED));
    }
}

std::string
arrays::h5_bridge::H5File::Impl::filename()
{
  return this->fname_;
}

void
arrays::h5_bridge::H5File::Impl::flush()
{
  H5Fflush(this->h5_->getId(), H5F_SCOPE_LOCAL);
}

std::string
arrays::h5_bridge::H5File::Impl::name(hid_t id)
{
  std::size_t len = H5Iget_name(id, NULL, 0);
  char buffer[len];
  H5Iget_name(id, buffer, len+1);
  std::string n = buffer;
  return n;
}

std::vector<std::string>
arrays::h5_bridge::H5File::Impl::subgroups(const std::string& path)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  std::vector<std::string> retval;
  if (! this->group(path, false))
    {
      return retval;
    }

  auto group = this->obj_cache_.at(path);

  try
    {
      (void) H5Literate(group->getId(),
                        H5_INDEX_NAME,
                        H5_ITER_INC,
                        nullptr,
                        [](hid_t g_id,
                           const char *name,
                           const H5L_info_t *info,
                           void *op_data) -> herr_t
                        {
                          std::vector<std::string> *v =
                            static_cast<std::vector<std::string>*>(op_data);
                          v->push_back(std::string(name));
                          return 0;
                        },
                        &retval);
    }
  catch (const H5::Exception& ex)
    {
      ALOG_ERROR("While enumerating subgroups from path {}: {}",
                 path, ex.getDetailMsg());
      throw(arrays::error_t(arrays::ERR_H5_EXCEPTION));
    }

  return retval;
}

std::vector<std::string>
arrays::h5_bridge::H5File::Impl::attributes(const std::string& path)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (! this->is_cached(path, H5I_BADID))
    {
      ALOG_ERROR("{} is not a cached object, instantiate it first", path);
      throw(arrays::error_t(arrays::ERR_H5_OBJ_NOT_IN_CACHE));
    }

  auto obj = this->obj_cache_.at(path);
  std::vector<std::string> retval;

  try
    {
      (void) H5Aiterate(obj->getId(),
                        H5_INDEX_NAME,
                        H5_ITER_INC,
                        nullptr,
                        [](hid_t loc_id,
                           const char *attr_name,
                           const H5A_info_t *ainf,
                           void *op_data) -> herr_t
                        {
                          std::vector<std::string> *v =
                            static_cast<std::vector<std::string>*>(op_data);
                          v->push_back(std::string(attr_name));
                          return 0;
                        },
                        &retval);
    }
  catch (const H5::Exception& ex)
    {
      ALOG_ERROR("While enumerating attributes from path {}: {}",
                 path, ex.getDetailMsg());
      throw(arrays::error_t(arrays::ERR_H5_EXCEPTION));
    }

  return retval;
}

bool
arrays::h5_bridge::H5File::Impl::is_cached(
  const std::string& path, const H5I_type_t tp)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  auto it = this->obj_cache_.find(path);
  if (it != this->obj_cache_.end())
    {
      H5ObjectPtr obj = this->obj_cache_.at(path);

      if (this->name(obj->getId()) == path)
        {
          if ((tp == H5I_BADID) || (obj->getHDFObjType() == tp))
            {
              return true;
            }
          else
            {
              ALOG_ERROR("Cached object {} is of incorrect type", path);
              throw(arrays::error_t(arrays::ERR_H5_INVALID_TYPE));
            }
        }
      else
        {
          this->obj_cache_.erase(it);
          obj.reset();
        }
    }

  return false;
}

bool
arrays::h5_bridge::H5File::Impl::group(const std::string& path, bool create)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (this->is_cached(path, H5I_GROUP))
    {
      return true;
    }

  H5GroupPtr group;
  std::string path_components;

  for (auto& component : fs::path(path))
    {
      if (component == "/")
        {
          path_components = "/";
        }
      else
        {
          path_components += std::string("/" + component.string());
        }

      try
        {
          group =
            std::make_shared<H5::Group>(this->h5_->openGroup(path_components));
        }
      catch (const H5::Exception& ex)
        {
          if (create)
            {
              try
                {
                  group =
                    std::make_shared<H5::Group>(
                      this->h5_->createGroup(path_components));
                }
              catch (const H5::Exception& ex)
                {
                  ALOG_ERROR("While trying to create group {}: {}",
                             path, ex.getDetailMsg());
                  throw(arrays::error_t(arrays::ERR_H5_CREATE_GROUP_FAILED));
                }
            }
          else
            {
              ALOG_ERROR("While trying to open group {}: {}",
                         path, ex.getDetailMsg());
              throw(arrays::error_t(arrays::ERR_H5_NO_SUCH_GROUP));
            }
        }
    }

  this->obj_cache_.insert({path, group});
  return true;
}

std::optional<H5AttrPtr>
arrays::h5_bridge::H5File::Impl::attr_obj(
  H5::H5Object *obj, const std::string& name)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  try
    {
      return std::make_shared<H5::Attribute>(obj->openAttribute(name));
    }
  catch (const H5::Exception& ex)
    {
      ALOG_WARN("No attribute '{}' on {}: {}",
                name, this->name(obj->getId()), ex.getDetailMsg());
      return std::nullopt;
    }
}

void
arrays::h5_bridge::H5File::Impl::attr(
  const std::string& path,
  const std::string& key,
  arrays::h5_bridge::Attr_t& attr_out)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (! this->is_cached(path, H5I_BADID))
    {
      ALOG_ERROR("{} is not a cached object, instantiate it first", path);
      throw(arrays::error_t(arrays::ERR_H5_OBJ_NOT_IN_CACHE));
    }

  auto obj = this->obj_cache_.at(path);
  std::optional<H5AttrPtr> attr = this->attr_obj(obj.get(), key);
  if (attr == std::nullopt)
    {
      ALOG_ERROR("Attribute {} on {} does not exist", key, path);
      throw(arrays::error_t(arrays::ERR_H5_NO_SUCH_ATTR));
    }

  H5::DataType dt = attr.value()->getDataType();
  std::visit([&attr, &dt](auto& val)
             {
               if (dt.detectClass(H5T_FLOAT))
                 {
                   attr.value()->read(attr.value()->getFloatType(), &val);
                 }
               else if (dt.detectClass(H5T_INTEGER))
                 {
                   attr.value()->read(attr.value()->getIntType(), &val);
                 }
               else if (dt.detectClass(H5T_STRING))
                 {
                   attr.value()->read(
                     attr.value()->getStrType(), *((std::string *) &val));
                 }
               else
                 {
                   ALOG_ERROR("Unsupported attribute type!");
                   throw(arrays::error_t(arrays::ERR_H5_INVALID_TYPE));
                 }
             }, attr_out);
}

void
arrays::h5_bridge::H5File::Impl::set_attr(
  const std::string& path, const std::string& key,
  const arrays::h5_bridge::Attr_t& value)
{
  if (! ARRAYS_H5_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (! this->is_cached(path, H5I_BADID))
    {
      ALOG_ERROR("{} is not a cached object, instantiate it first", path);
      throw(arrays::error_t(arrays::ERR_H5_OBJ_NOT_IN_CACHE));
    }

  auto obj = this->obj_cache_.at(path);
  std::optional<H5AttrPtr> attr;
  H5::DataSpace dspace(H5S_SCALAR);
  H5::DataType dtype = std::visit(H5DTypeVisitor(), value);

  attr = this->attr_obj(obj.get(), key);
  if (attr == std::nullopt)
    {
      try
        {
          attr = std::make_shared<H5::Attribute>(
            obj->createAttribute(key, dtype, dspace));
        }
      catch (const H5::Exception& ex)
        {
          ALOG_ERROR("Failed to create attribute: {} on {}: {}",
                     key, path, ex.getDetailMsg());
          throw(arrays::error_t(arrays::ERR_H5_CREATE_ATTR_FAILED));
        }
    }

  try
    {
      std::visit([&attr, &dtype](const auto& val)
                 { attr.value()->write(dtype, &val); }, value);
    }
  catch (const H5::Exception& ex)
    {
      throw(arrays::error_t(arrays::ERR_H5_WRITE_ATTR_FAILED));
    }
}

#endif // ARRAYS__H5_BRIDGE_H5_FILE_IMPL_H_
