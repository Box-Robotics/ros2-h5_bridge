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
#ifndef H5_BRIDGE__ERR_H_
#define H5_BRIDGE__ERR_H_

#include <exception>
#include <h5_bridge/visibility_control.h>

namespace h5_bridge
{
  extern H5_BRIDGE_PUBLIC const int OK;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_BAD_MODE;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_OPEN_FAILED;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_CREATE_GROUP_FAILED;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_EXCEPTION;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_NO_SUCH_GROUP;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_OBJ_NOT_IN_CACHE;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_INVALID_TYPE;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_NO_SUCH_ATTR;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_CREATE_ATTR_FAILED;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_WRITE_ATTR_FAILED;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_OPEN_DSET_FAILED;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_CREATE_DSET_FAILED;
  extern H5_BRIDGE_PUBLIC const int ERR_H5_WRITE_DSET_FAILED;

  /**
   * Human-readable string representation of an error code
   *
   * @param[in] errnum The error code to convert to a string
   * @return A string version of the error code.
   */
  const char *strerror(int errnum);

  /**
   * Exception wrapper for error codes
   */
  class H5_BRIDGE_PUBLIC error_t : public std::exception
  {
  public:
    /**
     * Constructs an error_t (exception) from an error code
     */
    error_t(int errnum);

    /**
     * Retrieves the exception message
     */
    virtual const char *what() const noexcept;

    /**
     * Accessor to the internally wrapped error code
     */
    int code() const noexcept;

  private:
    int errnum_;

  }; // end: class error_t

} // end: namespace h5_bridge

#endif // H5_BRIDGE__ERR_H_
