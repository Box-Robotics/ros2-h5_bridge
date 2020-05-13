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
#ifndef H5_BRIDGE__H5_BRIDGE_H5_FILE_IMPL_H_
#define H5_BRIDGE__H5_BRIDGE_H5_FILE_IMPL_H_

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <H5Cpp.h>
#include <h5_bridge/err.hpp>
#include <h5_bridge/logging.hpp>
#include <h5_bridge/h5_attr_t.hpp>
#include <h5_bridge/h5_dset_t.hpp>

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

const static int H5_BRIDGE_VERBOSE =
  std::getenv("H5_BRIDGE_VERBOSE") == nullptr ? 0 : 1;

//-----------------------------
// Impl interface
//-----------------------------

namespace h5_bridge
{
  class H5File::Impl
  {
  public:
    Impl(const std::string& path, const std::string& mode);
    ~Impl() = default;

    std::string filename();
    bool group(const std::string& path, bool create = true);
    bool dset(const std::string& path);
    std::vector<std::string> subgroups(const std::string& path);
    std::vector<std::string> attributes(const std::string& path);
    std::vector<std::string> datasets(const std::string& path);
    void set_attr(const std::string& path, const std::string& key,
                  const h5_bridge::Attr_t& value);
    void attr(const std::string& path, const std::string& key,
              h5_bridge::Attr_t& attr_out);
    void write(const std::string& path, const std::uint8_t * buff,
               const h5_bridge::DSet_t dset_t, int rows, int cols, int chans,
               int gzip);
    std::tuple<h5_bridge::DSet_t, int, int, int>
    read(const std::string& path, std::uint8_t * buff = nullptr);
    void clear(const std::string& path);
    void clear_all();
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

} // end: namespace h5_bridge

//-----------------------------
// Impl implementation
//-----------------------------

h5_bridge::H5File::Impl::Impl(
  const std::string& path, const std::string& mode)
  : fname_(path)
{
  if (! H5_BRIDGE_VERBOSE)
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
          H5B_ERROR("Unrecoginized H5 open mode: {}", mode);
          throw(h5_bridge::error_t(h5_bridge::ERR_H5_BAD_MODE));
        }
    }
  catch (const H5::Exception& ex)
    {
      H5B_ERROR("Failed to open {} with mode {}: {}",
                path, mode, ex.getDetailMsg());
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_OPEN_FAILED));
    }
}

std::string
h5_bridge::H5File::Impl::filename()
{
  return this->fname_;
}

void
h5_bridge::H5File::Impl::clear(const std::string& path)
{
  this->obj_cache_.erase(path);
}

void
h5_bridge::H5File::Impl::clear_all()
{
  this->obj_cache_.clear();
}

void
h5_bridge::H5File::Impl::flush()
{
  H5Fflush(this->h5_->getId(), H5F_SCOPE_LOCAL);
}

std::string
h5_bridge::H5File::Impl::name(hid_t id)
{
  std::size_t len = H5Iget_name(id, NULL, 0);
  char buffer[len];
  H5Iget_name(id, buffer, len+1);
  std::string n = buffer;
  return n;
}

std::vector<std::string>
h5_bridge::H5File::Impl::datasets(const std::string& path)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  std::vector<std::string> retval;
  if (! this->group(path, false))
    {
      return retval;
    }

  auto group = this->obj_cache_.at(path);
  std::vector<std::tuple<int, int, std::string>> elem_vec;

  try
    {
      (void) this->h5_->iterateElems(
        path.c_str(),
        nullptr,
        [](hid_t loc_id,
           const char *name,
           void *op_data) -> herr_t
        {
          auto *v =
            static_cast<std::vector<std::tuple<int,int,std::string>>*>(op_data);

          int idx = v->size();
          int obj_type = H5Gget_objtype_by_idx(loc_id, idx);
          v->push_back(std::make_tuple(idx, obj_type, std::string(name)));

          return 0;
        },
        &elem_vec);
    }
  catch (const H5::Exception& ex)
    {
      H5B_ERROR("While enumerating datasets from path {}: {}",
                path, ex.getDetailMsg());
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_EXCEPTION));
    }

  for (auto& tup : elem_vec)
    {
      if (std::get<1>(tup) == H5G_DATASET)
        {
          retval.push_back(std::get<2>(tup));
        }
    }

  return retval;
}

std::vector<std::string>
h5_bridge::H5File::Impl::subgroups(const std::string& path)
{
  if (! H5_BRIDGE_VERBOSE)
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
      H5B_ERROR("While enumerating subgroups from path {}: {}",
                path, ex.getDetailMsg());
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_EXCEPTION));
    }

  return retval;
}

std::vector<std::string>
h5_bridge::H5File::Impl::attributes(const std::string& path)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (! this->is_cached(path, H5I_BADID))
    {
      H5B_ERROR("{} is not a cached object, instantiate it first", path);
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_OBJ_NOT_IN_CACHE));
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
      H5B_ERROR("While enumerating attributes from path {}: {}",
                path, ex.getDetailMsg());
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_EXCEPTION));
    }

  return retval;
}

bool
h5_bridge::H5File::Impl::is_cached(
  const std::string& path, const H5I_type_t tp)
{
  if (! H5_BRIDGE_VERBOSE)
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
              H5B_ERROR("Cached object {} is of incorrect type", path);
              throw(h5_bridge::error_t(h5_bridge::ERR_H5_INVALID_TYPE));
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
h5_bridge::H5File::Impl::dset(const std::string& path)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (this->is_cached(path, H5I_DATASET))
    {
      return true;
    }

  H5DSetPtr dset;

  try
    {
      dset = std::make_shared<H5::DataSet>(this->h5_->openDataSet(path));
    }
  catch (const H5::Exception& ex)
    {
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_OPEN_DSET_FAILED));
    }

  this->obj_cache_.insert({path, dset});
  return true;
}

bool
h5_bridge::H5File::Impl::group(const std::string& path, bool create)
{
  if (! H5_BRIDGE_VERBOSE)
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
                  H5B_ERROR("While trying to create group {}: {}",
                            path, ex.getDetailMsg());
                  throw(h5_bridge::error_t(
                          h5_bridge::ERR_H5_CREATE_GROUP_FAILED));
                }
            }
          else
            {
              H5B_ERROR("While trying to open group {}: {}",
                        path, ex.getDetailMsg());
              throw(h5_bridge::error_t(h5_bridge::ERR_H5_NO_SUCH_GROUP));
            }
        }
    }

  this->obj_cache_.insert({path, group});
  return true;
}

std::optional<H5AttrPtr>
h5_bridge::H5File::Impl::attr_obj(H5::H5Object *obj, const std::string& name)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  try
    {
      return std::make_shared<H5::Attribute>(obj->openAttribute(name));
    }
  catch (const H5::Exception& ex)
    {
      H5B_WARN("No attribute '{}' on {}: {}",
               name, this->name(obj->getId()), ex.getDetailMsg());
      return std::nullopt;
    }
}

void
h5_bridge::H5File::Impl::attr(
 const std::string& path,
 const std::string& key,
 h5_bridge::Attr_t& attr_out)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (! this->is_cached(path, H5I_BADID))
    {
      H5B_ERROR("{} is not a cached object, instantiate it first", path);
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_OBJ_NOT_IN_CACHE));
    }

  auto obj = this->obj_cache_.at(path);
  std::optional<H5AttrPtr> attr = this->attr_obj(obj.get(), key);
  if (attr == std::nullopt)
    {
      H5B_ERROR("Attribute {} on {} does not exist", key, path);
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_NO_SUCH_ATTR));
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
                   H5B_ERROR("Unsupported attribute type!");
                   throw(h5_bridge::error_t(h5_bridge::ERR_H5_INVALID_TYPE));
                 }
             }, attr_out);
}

void
h5_bridge::H5File::Impl::set_attr(
 const std::string& path, const std::string& key,
 const h5_bridge::Attr_t& value)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  if (! this->is_cached(path, H5I_BADID))
    {
      H5B_ERROR("{} is not a cached object, instantiate it first", path);
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_OBJ_NOT_IN_CACHE));
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
          H5B_ERROR("Failed to create attribute: {} on {}: {}",
                    key, path, ex.getDetailMsg());
          throw(h5_bridge::error_t(h5_bridge::ERR_H5_CREATE_ATTR_FAILED));
        }
    }

  try
    {
      std::visit([&attr, &dtype](const auto& val)
                 { attr.value()->write(dtype, &val); }, value);
    }
  catch (const H5::Exception& ex)
    {
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_WRITE_ATTR_FAILED));
    }
}

void
h5_bridge::H5File::Impl::write(
  const std::string& path, const std::uint8_t * buff,
  const h5_bridge::DSet_t dset_t,
  int rows, int cols, int chans, int gzip)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  hsize_t maxdims[3] = {H5S_UNLIMITED, H5S_UNLIMITED, H5S_UNLIMITED};
  hsize_t dims[3] = {static_cast<hsize_t>(rows),
                     static_cast<hsize_t>(cols),
                     static_cast<hsize_t>(chans)};

  int rank = (chans == 1 ? 2 : 3);
  H5::DataSpace dspace(rank, dims, maxdims);
  H5::DataType dtype = std::visit(H5DTypeVisitor(), dset_t);

  H5DSetPtr dset;

  try
    {
      if (this->dset(path))
        {
          dset =
            std::dynamic_pointer_cast<H5::DataSet>(this->obj_cache_.at(path));
          dset->extend(dims);
        }
    }
  catch (const h5_bridge::error_t& ex)
    {
      try
        {
          H5::DSetCreatPropList prop;
          prop.setChunk(rank, dims);
          if ((gzip > 0) && (gzip <= 9))
            {
              prop.setDeflate(gzip);
            }

          if (! this->group(fs::path(path).remove_filename().string()))
            {
              H5B_ERROR("Could not create path to new dataset: {}",
                        path);
              throw(h5_bridge::error_t(h5_bridge::ERR_H5_CREATE_DSET_FAILED));
            }

          dset = std::make_shared<H5::DataSet>(
            this->h5_->createDataSet(path, dtype, dspace, prop));
        }
      catch (const H5::Exception& ex)
        {
          H5B_ERROR("Failed to create dataset: {}: {}",
                    path, ex.getDetailMsg());
          throw(h5_bridge::error_t(h5_bridge::ERR_H5_CREATE_DSET_FAILED));
        }
    }

  try
    {
      dset->write(buff, dtype);
    }
  catch (const H5::Exception& ex)
    {
      H5B_ERROR("Could not write dataset {}: {}", path, ex.getDetailMsg());
      throw(h5_bridge::error_t(h5_bridge::ERR_H5_WRITE_DSET_FAILED));
    }

  this->obj_cache_.insert({path, dset});
}

std::tuple<h5_bridge::DSet_t, int, int, int>
h5_bridge::H5File::Impl::read(const std::string& path, std::uint8_t * buff)
{
  if (! H5_BRIDGE_VERBOSE)
    {
      H5::Exception::dontPrint();
    }

  H5DSetPtr dset;
  if (this->dset(path))
    {
      dset =
        std::dynamic_pointer_cast<H5::DataSet>(this->obj_cache_.at(path));
    }

  H5::DataType dt = dset->getDataType();
  h5_bridge::DSet_t tp;

  if (dt == H5::PredType::NATIVE_UINT8)
    {
      tp = std::uint8_t{0};
    }
  else if (dt == H5::PredType::NATIVE_INT8)
    {
      tp = std::int8_t{0};
    }
  else if (dt == H5::PredType::NATIVE_UINT16)
    {
      tp = std::uint16_t{0};
    }
  else if (dt == H5::PredType::NATIVE_INT16)
    {
      tp = std::int16_t{0};
    }
  else if (dt == H5::PredType::NATIVE_INT32)
    {
      tp = std::int32_t{0};
    }
  else if (dt == H5::PredType::NATIVE_FLOAT)
    {
      tp = float{0.0};
    }
  else if (dt == H5::PredType::NATIVE_DOUBLE)
    {
      tp = double{0.0};
    }

  H5::DataSpace file_space = dset->getSpace();
  hsize_t dims_out[file_space.getSimpleExtentNdims()];
  int rank = file_space.getSimpleExtentDims(dims_out, nullptr);

  int nchans = 1;
  if (rank == 3)
    {
      nchans = dims_out[2];
    }
  int rows = dims_out[0];
  int cols = rank == 1 ? 1 : dims_out[1];

  if (buff != nullptr)
    {
      dset->read(buff,
                 dt,
                 H5::DataSpace(rank, dims_out),
                 file_space);
    }

  dset->close();
  return std::make_tuple(tp, rows, cols, nchans);
}

#endif // ARRAYS__H5_BRIDGE_H5_FILE_IMPL_H_
