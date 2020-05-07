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
#ifndef ARRAYS__LOGGING_H_
#define ARRAYS__LOGGING_H_

#define SPDLOG_ACTIVE_LEVEL 0

#include <memory>
#include <string>
#include <spdlog/spdlog.h>
#include <arrays/visibility_control.h>

inline std::string const ARRAYS_LOGGER_NAME = "arrays";
inline std::string const ARRAYS_LOGGER_FORMAT =
  "[%Y-%m-%d %H:%M:%S.%f][%t][%L][%s:%#] %v";

namespace arrays
{
  /**
   * @namespace arrays::log
   *
   * An alias for `spdlog` namespace. So, any call to `spdlog::XXX` can be
   * replaced with `arrays::log::XXX`
   */
  namespace log = spdlog;

  /**
   * Returns the library's default logger
   *
   * @return A `std::shared_ptr` to the logger.
   */
  ARRAYS_PUBLIC std::shared_ptr<arrays::log::logger>
  get_default_logger();

  /**
   * Sets the library's default logger
   *
   * @param[in] logger The logger to set as the library default.
   */
  ARRAYS_PUBLIC void
  set_default_logger(std::shared_ptr<arrays::log::logger> logger);

  /**
   * This is an in-line convenience function that allows library consumers to
   * send their log messages through to the same logger as the library itself.
   *
   * If this function is not called by consumers, log messages will go to a
   * default logger unique to the library or binary that they are linked
   * to. This is because `spdlog` does not share registries across shared
   * object boundaries. More info
   * [here](https://github.com/gabime/spdlog/wiki/How-to-use-spdlog-in-DLLs).
   *
   * The general guidance is, call this function once in your library or
   * program, then use the `ALOG_XXX` macros as usual to emit log messages.
   */
  inline void
  use_arrays_logger()
  { spdlog::set_default_logger(arrays::get_default_logger()); }

} // end: namespace arrays

#define ALOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define ALOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define ALOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define ALOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define ALOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define ALOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

#endif // ARRAYS__LOGGING_H_
