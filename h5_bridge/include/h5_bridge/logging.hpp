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
#ifndef H5_BRIDGE__LOGGING_H_
#define H5_BRIDGE__LOGGING_H_

#define SPDLOG_ACTIVE_LEVEL 0

#include <memory>
#include <string>
#include <spdlog/spdlog.h>
#include <h5_bridge/visibility_control.h>

inline std::string const H5_BRIDGE_LOGGER_NAME = "h5_bridge";
inline std::string const H5_BRIDGE_LOGGER_FORMAT =
  "[%Y-%m-%d %H:%M:%S.%f][%t][%L][%s:%#] %v";

namespace h5_bridge
{
  /**
   * @namespace h5_bridge::log
   *
   * An alias for `spdlog` namespace. So, any call to `spdlog::XXX` can be
   * replaced with `h5_bridge::log::XXX`
   */
  namespace log = spdlog;

  /**
   * Returns the library's default logger
   *
   * @return A `std::shared_ptr` to the logger.
   */
  H5_BRIDGE_PUBLIC std::shared_ptr<h5_bridge::log::logger>
  get_default_logger();

  /**
   * Sets the library's default logger
   *
   * @param[in] logger The logger to set as the library default.
   */
  H5_BRIDGE_PUBLIC void
  set_default_logger(std::shared_ptr<h5_bridge::log::logger> logger);

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
   * program, then use the `H5B_XXX` macros as usual to emit log messages.
   */
  inline void
  use_h5_bridge_logger()
  { spdlog::set_default_logger(h5_bridge::get_default_logger()); }

} // end: namespace h5_bridge

#define H5B_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define H5B_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define H5B_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define H5B_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define H5B_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define H5B_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

#endif // H5_BRIDGE__LOGGING_H_
