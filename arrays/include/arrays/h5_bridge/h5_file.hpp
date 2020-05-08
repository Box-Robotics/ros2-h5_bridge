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
#ifndef ARRAYS__H5_BRIDGE_H5_FILE_H_
#define ARRAYS__H5_BRIDGE_H5_FILE_H_

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include <arrays/h5_bridge/h5_attr_t.hpp>
#include <arrays/visibility_control.h>

namespace arrays::h5_bridge
{
  /**
   * The H5 resources managed by a file container (see `H5File`) that we
   * primarily care about are: groups, datasets, and attributes. For the
   * former two, groups and datasets, just like the Unix filesystem, they can
   * be uniquely identified as an absolute path and encoded as a string. In
   * terms of attributes, within the context of the object they are annotating
   * (group or dataset), they can also be uniquely identified by a string (like
   * a key in a hashmap or dictionary). To that end, to not leak the underlying
   * H5 library structures to our users, we represent H5 resources as a
   * `std::optional<std::string>`. We use the optional for the cases when a
   * return value from a function may have to, for example, look up a resource,
   * but it does not exist. In that case the optional can store a
   * `std::nullopt`.
   */
  using H5ObjId = std::optional<std::string>;

  /**
   * The `H5File` class implements a proxy interface to an underlying HDF5 file
   * on disk. This class tries to abstract away the underlying details of (the
   * very complex) HDF5 specification. Additionally, it is a design choice to
   * not leak any native H5 data types into a consuming application. This is
   * meant to be "H5 for humans" ... human C++ programmers :-)
   *
   * Given that H5 files effectively act like a "filesystem in a file," within
   * the `arrays` library, we are mostly interested in being able to store
   * arrays ("data sets" in H5-speak)  into and load arrays from H5 files. We
   * also want to organize them into groups (filesystem-like paths) and well as
   * provide `key=value` attributes on groups and arrays.
   */
  class ARRAYS_PUBLIC H5File
  {
  public:
    using Ptr = std::unique_ptr<H5File>;
    using SharedPtr = std::shared_ptr<H5File>;

    /**
     * Provides access to an `H5File` container by either opening an existing
     * file or creating a new one.
     *
     * @param[in] path The full filesystem path to the file to open
     * @param[in] mode The mode to open the file in. Allowed modes are:
     *   "r" Readonly, file must exist
     *   "r+" Read/write, file must exist
     *   "w" Create file, truncate if exists
     *   "w-" or 'x' Create file, fail if exists
     *   "a" Read/write if exists, create otherwise (default)
     */
    explicit H5File(const std::string& path, const std::string& mode = "a");

    /**
     * Ctor overload that takes a `std::filesystem::path`. Semantics are the
     * same as the `std::string` ctor.
     */
    explicit H5File(const std::filesystem::path& path,
                    const std::string& mode = "a");

    /**
     * RAII deallocs
     */
    virtual ~H5File();

    // `H5File` is a move-only type, so we explicitly disable copy-semantics
    H5File(const H5File&) = delete;
    H5File& operator=(const H5File&) = delete;

    // Default implementation of the move operations does an element-by-element
    // move. Which is exactly what we want since we are PIMPL. But, we have to
    // do the `= default` in the `.cpp` file b/c our `pImpl` is of incomplete
    // type `Impl`. In the `.cpp`, we have the impl header available to us.
    H5File(H5File&&);
    H5File& operator=(H5File&&);

    /**
     * Accessor to the underlying filename we are manipulating.
     */
    std::string filename();

    /**
     * Creates a new group at the specified `path`. If the group already
     * exists, it simply returns the `H5ObjId` pointing to the existing
     * group. If the group could not be created, the `H5ObjId` will hold a
     * `std::nullopt`.
     *
     * @param[in] path Full path the group to create or query.
     * @return An `H5ObjId` pointing to the group.
     */
    arrays::h5_bridge::H5ObjId group(const std::string& path);

    /**
     * Provide a list of immediate subgroups of the passed-in group. This does
     * not recurse beyond the next level down the tree from the passed in
     * group. If the passed in `group` is `std::nullopt`, it will return the
     * subgroups of "/";
     *
     * @param[in] group The `H5ObjId` pointing to the group whose first-level
     *                  subgroups we want a listing of.
     * @return A vector of strings of the names of the first-level subgroups of
     *                  `group`.
     */
    std::vector<std::string> subgroups(const arrays::h5_bridge::H5ObjId& group);

    /**
     * Provides a list of attribute names that are annotating the passed in
     * `obj`. If the `obj` is not currently cached (i.e., instantiated via
     * something like a call to `group(...) or loading a data set explicitly)
     * an exception will be thrown. If the passed in `obj` is `std::nullopt`,
     * it will try to get the attributes of "/" which is almost certainly not
     * what you want and will almost certainly result in an exception being
     * thrown since it is unlikely that "/" is cached.
     *
     * @param[in] obj The cached objects whose attribute list we want to
     *                retrieve.
     * @return A vector of strings of the names of the attributes annotating
     *                `obj`.
     */
    std::vector<std::string> attributes(const arrays::h5_bridge::H5ObjId& obj);

    /**
     * Annotates an object `obj` with a scalar attribute
     *
     * @param[in] obj The object we wish to annotate
     * @param[in] key The attribute key
     * @param[in] value The value of the attribute
     */
    void set_attr(const arrays::h5_bridge::H5ObjId& obj,
                  const std::string& key,
                  const arrays::h5_bridge::Attr_t& value);

    /**
     * Special-case for writing an attribute on to an object whose value is a
     * C-string.
     *
     * @param[in] obj The object we wish to annotate
     * @param[in] key The attribute key
     * @param[in] value A C-string value
     */
    void set_attr(const arrays::h5_bridge::H5ObjId& obj,
                  const std::string& key,
                  const char * value);

    /**
     * Accessor for the value of an attribute on a cached object.
     *
     * @param[in] obj The cached object whose attribute we want to access
     * @param[in] key The attribute name
     *
     * @return The value of the attribute.
     */
    template<typename T>
    T attr(const arrays::h5_bridge::H5ObjId& obj, const std::string& key)
    {
      arrays::h5_bridge::Attr_t value;
      if constexpr (std::is_same_v<T, std::string>)
        {
          value.emplace<T>(std::string(""));
        }
      else
        {
          value.emplace<T>(T{0});
        }

      this->attr(obj, key, value);
      T retval = std::get<T>(value);
      return retval;
    }

    /**
     * Flush buffers to disk
     */
    void flush();

  private:
    void attr(const arrays::h5_bridge::H5ObjId& obj, const std::string& key,
              arrays::h5_bridge::Attr_t& value_out);

    class Impl;
    std::unique_ptr<Impl> pImpl;

  }; // end: class H5File

} // end: namespace arrays::h5_bridge

#endif // ARRAYS__H5_BRIDGE_H5_FILE_H_
