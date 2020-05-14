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

#include <h5_bridge/err.hpp>
#include <cstring>
#include <stdexcept>

const int h5_bridge::OK = 0;
const int h5_bridge::ERR_H5_BAD_MODE = 6000;
const int h5_bridge::ERR_H5_OPEN_FAILED = 6001;
const int h5_bridge::ERR_H5_CREATE_GROUP_FAILED = 6002;
const int h5_bridge::ERR_H5_EXCEPTION = 6003;
const int h5_bridge::ERR_H5_NO_SUCH_GROUP = 6004;
const int h5_bridge::ERR_H5_OBJ_NOT_IN_CACHE = 6005;
const int h5_bridge::ERR_H5_INVALID_TYPE = 6006;
const int h5_bridge::ERR_H5_NO_SUCH_ATTR = 6007;
const int h5_bridge::ERR_H5_CREATE_ATTR_FAILED = 6008;
const int h5_bridge::ERR_H5_WRITE_ATTR_FAILED = 6009;
const int h5_bridge::ERR_H5_OPEN_DSET_FAILED = 6010;
const int h5_bridge::ERR_H5_CREATE_DSET_FAILED = 6011;
const int h5_bridge::ERR_H5_WRITE_DSET_FAILED = 6012;
const int h5_bridge::ERR_H5_BAD_SHAPE = 6013;
const int h5_bridge::ERR_H5_CORRUPT_FILE = 6014;

const char *h5_bridge::strerror(int errnum)
{
  switch (errnum)
    {
    case h5_bridge::OK:
      return "OK";

    case h5_bridge::ERR_H5_BAD_MODE:
      return "Bad H5 file mode";

    case h5_bridge::ERR_H5_OPEN_FAILED:
      return "Failed to open H5 file with passed in mode, check permissions";

    case h5_bridge::ERR_H5_CREATE_GROUP_FAILED:
      return "Failed to create Group";

    case h5_bridge::ERR_H5_EXCEPTION:
      return "The underlying H5 library encountered an error";

    case h5_bridge::ERR_H5_NO_SUCH_GROUP:
      return "The H5 group does not exist";

    case h5_bridge::ERR_H5_OBJ_NOT_IN_CACHE:
      return "Requested object is not cached, instantiate it first";

    case h5_bridge::ERR_H5_INVALID_TYPE:
      return "The H5Object is of the wrong type";

    case h5_bridge::ERR_H5_NO_SUCH_ATTR:
      return "The H5 object has no such attribute";

    case h5_bridge::ERR_H5_CREATE_ATTR_FAILED:
      return "Failed to create attribute";

    case h5_bridge::ERR_H5_WRITE_ATTR_FAILED:
      return "Failed to write attribute value";

    case h5_bridge::ERR_H5_OPEN_DSET_FAILED:
      return "Failed to open DataSet";

    case h5_bridge::ERR_H5_CREATE_DSET_FAILED:
      return "Failed to create DataSet";

    case h5_bridge::ERR_H5_WRITE_DSET_FAILED:
      return "Failed to write DataSet";

    case h5_bridge::ERR_H5_BAD_SHAPE:
      return "The data has an invalid shape";

    case h5_bridge::ERR_H5_CORRUPT_FILE:
      return "The logical structure of the data appears to be corrupt";

    default:
      return ::strerror(errnum);
    }
}

h5_bridge::error_t::error_t(int errnum)
  : std::exception(), errnum_(errnum) {}

int h5_bridge::error_t::code() const noexcept
{
  return this->errnum_;
}

const char *h5_bridge::error_t::what() const noexcept
{
  return h5_bridge::strerror(this->code());
}
