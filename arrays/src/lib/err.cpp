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

#include <arrays/err.hpp>
#include <cstring>
#include <stdexcept>

const int arrays::OK = 0;
const int arrays::ERR_H5_BAD_MODE = 6000;
const int arrays::ERR_H5_OPEN_FAILED = 6001;
const int arrays::ERR_H5_CREATE_GROUP_FAILED = 6002;
const int arrays::ERR_H5_EXCEPTION = 6003;
const int arrays::ERR_H5_NO_SUCH_GROUP = 6004;
const int arrays::ERR_H5_OBJ_NOT_IN_CACHE = 6005;
const int arrays::ERR_H5_INVALID_TYPE = 6006;
const int arrays::ERR_H5_NO_SUCH_ATTR = 6007;
const int arrays::ERR_H5_CREATE_ATTR_FAILED = 6008;
const int arrays::ERR_H5_WRITE_ATTR_FAILED = 6009;

const char *arrays::strerror(int errnum)
{
  switch (errnum)
    {
    case arrays::OK:
      return "OK";

    case arrays::ERR_H5_BAD_MODE:
      return "Bad H5 file mode";

    case arrays::ERR_H5_OPEN_FAILED:
      return "Failed to open H5 file with passed in mode, check permissions";

    case arrays::ERR_H5_CREATE_GROUP_FAILED:
      return "Failed to create Group";

    case arrays::ERR_H5_EXCEPTION:
      return "The underlying H5 library encountered an error";

    case arrays::ERR_H5_NO_SUCH_GROUP:
      return "The H5 group does not exist";

    case arrays::ERR_H5_OBJ_NOT_IN_CACHE:
      return "Requested object is not cached, instantiate it first";

    case arrays::ERR_H5_INVALID_TYPE:
      return "The H5Object is of the wrong type";

    case arrays::ERR_H5_NO_SUCH_ATTR:
      return "The H5 object has no such attribute";

    case arrays::ERR_H5_CREATE_ATTR_FAILED:
      return "Failed to create attribute";

    case arrays::ERR_H5_WRITE_ATTR_FAILED:
      return "Failed to write attribute value";

    default:
      return ::strerror(errnum);
    }
}

arrays::error_t::error_t(int errnum)
  : std::exception(), errnum_(errnum) {}

int arrays::error_t::code() const noexcept
{
  return this->errnum_;
}

const char *arrays::error_t::what() const noexcept
{
  return arrays::strerror(this->code());
}
