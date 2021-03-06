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
#include <h5_bridge/logging.hpp>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <system_error>
#include <thread>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace h5_bridge
{
  /**
   * Wrapper used to initialize the underlying default spdlog logger
   */
  class Logging
  {
  private:
    /**
     * Flag indicating the initialization state of the logging subsystem
     */
    static std::once_flag init_;

    /**
     * Runs the one-time setup of the spdlog default logger
     *
     * (parameters are the same as the public `Init` method.
     */
    static void _Init(int level,
                      int flush_level,
                      const std::string& format,
                      const std::string& log_dir,
                      const std::string& log_file,
                      std::size_t max_file_size,
                      std::size_t max_files,
                      std::size_t async_queue_size,
                      std::size_t nthreads)
    {
      spdlog::init_thread_pool(async_queue_size, nthreads);
      fs::path outfile =
        h5_bridge::Logging::construct_logfile_path_(log_dir, log_file);
      fs::path dir_part = outfile.parent_path();
      fs::path file_part = outfile.filename();

      std::error_code ec;
      fs::create_directories(dir_part, ec);
      auto logger = ec ?
        spdlog::create_async_nb<spdlog::sinks::stdout_color_sink_mt>(
          H5_BRIDGE_LOGGER_NAME) :
        spdlog::create_async_nb<spdlog::sinks::rotating_file_sink_mt>(
              H5_BRIDGE_LOGGER_NAME,
              outfile.u8string(),
              max_file_size,
              max_files);

      logger->set_level(static_cast<spdlog::level::level_enum>(level));
      logger->flush_on(static_cast<spdlog::level::level_enum>(flush_level));
      logger->set_pattern(format);
      spdlog::set_default_logger(logger);
    }

    /**
     * Here we construct the absolute file path to our logger. The rules are:
     *
     * 1. If `file` is not specified, we contruct a new file name relative to
     *    `dir`. Our filename will be named:
     *
     *     H5_BRIDGE_LOGGER_NAME.<timestamp>.<thread_id>.log
     *
     * 2. If `file` is specified:
     *    2.1. If it is an absolute path, we ignore `dir` and log to the
     *         absolute file path.
     *    2.2. If it is a relative path, we construct a filename relative to
     *         `dir`.
     */
    static fs::path construct_logfile_path_(const std::string& dir,
                                            const std::string& file)
    {
      fs::path path(dir);

      if (file.empty())
        {
          std::ostringstream os;
          os << H5_BRIDGE_LOGGER_NAME << "."
             << std::time(nullptr) << "."
             << std::this_thread::get_id() << "."
             << "log";

          path /= os.str();
        }
      else
        {
          fs::path file_part(file);
          if (file_part.is_absolute())
            {
              path = file_part;
            }
          else
            {
              path /= file;
            }
        }

      return path;
    }

  public:
    /**
     * Function used to initialize the library's logging subsystem. This
     * function is both thread-safe and idempotent.
     *
     * @param[in] level The default logging level
     * @param[in] flush_level The log level that triggers a flush to disk
     * @param[in] format The log message format string
     * @param[in] log_dir The directory into which we should write our logs
     * @param[in] log_file The path to the log file to write.
     * @param[in] max_file_size The maximum file size (bytes) to write when
     *                          creating a rotating log file.
     * @param[in] max_files The maximum number of historical log files to keep
     *                      around when using a rotating file logger.
     * @param[in] async_queue_size The maximum number of log messages to queue
     *                             up before discarding old log messages.
     * @param[in] nthreads The number of backing threads for writing the queued
     *                     up log messages to disk.
     */
    static void Init(int level,
                     int flush_level,
                     const std::string& format,
                     const std::string& log_dir,
                     const std::string& log_file,
                     std::size_t max_file_size,
                     std::size_t max_files,
                     std::size_t async_queue_size,
                     std::size_t nthreads)
    {
      std::call_once(h5_bridge::Logging::init_, h5_bridge::Logging::_Init,
                     level,
                     flush_level,
                     format,
                     log_dir,
                     log_file,
                     max_file_size,
                     max_files,
                     async_queue_size,
                     nthreads);
    }
  }; // end: class Logging

  std::shared_ptr<spdlog::logger> get_default_logger()
  {
    return spdlog::default_logger();
  }

  void set_default_logger(std::shared_ptr<h5_bridge::log::logger> logger)
  {
    spdlog::set_default_logger(std::move(logger));
  }

} // end: namespace h5_bridge

std::once_flag h5_bridge::Logging::init_;

// Initializer sample for MSVC and GCC/Clang.
// 2010-2016 Joe Lowe. Released into the public domain.
#ifdef __cplusplus
    #define INITIALIZER(f) \
        static void f(void); \
        struct f##_t_ { f##_t_(void) { f(); } }; static f##_t_ f##_; \
        static void f(void)
#elif defined(_MSC_VER)
    #pragma section(".CRT$XCU",read)
    #define INITIALIZER2_(f,p) \
        static void f(void); \
        __declspec(allocate(".CRT$XCU")) void (*f##_)(void) = f; \
        __pragma(comment(linker,"/include:" p #f "_")) \
        static void f(void)
    #ifdef _WIN64
        #define INITIALIZER(f) INITIALIZER2_(f,"")
    #else
        #define INITIALIZER(f) INITIALIZER2_(f,"_")
    #endif
#else
    #define INITIALIZER(f) \
        static void f(void) __attribute__((constructor)); \
        static void f(void)
#endif

INITIALIZER(h5_bridge_ctor)
{
  std::string yaml = "";
  bool yaml_ok = false;
  YAML::Node root_node;

  try
    {
      auto share_dir =
        ament_index_cpp::get_package_share_directory("h5_bridge");
      auto config_file = share_dir + "/etc/h5_bridge_logging.yaml";

      if (fs::exists(config_file))
        {
          std::ifstream ifs(config_file, std::ios::in);
          yaml.assign((std::istreambuf_iterator<char>(ifs)),
                      (std::istreambuf_iterator<char>()));
          yaml_ok = true;
        }

      root_node = YAML::Load(yaml);
    }
  catch (const ament_index_cpp::PackageNotFoundError& ex)
    {
      yaml_ok = false;
    }
  catch (const std::exception & ex)
    {
      yaml_ok = false;
    }


  int log_level = spdlog::level::info;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_LEVEL") == nullptr)
        {
          if (yaml_ok && root_node["log_level"])
            {
              log_level = root_node["log_level"].as<int>();
            }
        }
      else
        {
          log_level = std::atoi(std::getenv("H5_BRIDGE_LOG_LEVEL"));
        }
    }
  catch (const std::exception& ex)
    { }

  int log_flush_level = spdlog::level::trace;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_FLUSH_LEVEL") == nullptr)
        {
          if (yaml_ok && root_node["flush_level"])
            {
              log_flush_level = root_node["flush_level"].as<int>();
            }
        }
      else
        {
          log_flush_level = std::atoi(std::getenv("H5_BRIDGE_LOG_FLUSH_LEVEL"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::string log_dir = "/tmp";
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_DIR") == nullptr)
        {
          if (yaml_ok && root_node["log_dir"])
            {
              log_dir = root_node["log_dir"].as<std::string>();
            }
        }
      else
        {
          log_dir = std::string(std::getenv("H5_BRIDGE_LOG_DIR"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::string log_file = "";
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_FILE") == nullptr)
        {
          if (yaml_ok && root_node["log_file"])
            {
              log_file = root_node["log_file"].as<std::string>();
            }
        }
      else
        {
          log_file = std::string(std::getenv("H5_BRIDGE_LOG_FILE"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::size_t log_max_file_size = 1024*1024*5;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_MAX_FILE_SIZE") == nullptr)
        {
          if (yaml_ok && root_node["max_file_size"])
            {
              log_max_file_size = root_node["max_file_size"].as<std::size_t>();
            }
        }
      else
        {
          log_max_file_size =
            std::atoi(std::getenv("H5_BRIDGE_LOG_MAX_FILE_SIZE"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::size_t log_max_files = 5;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_MAX_FILES") == nullptr)
        {
          if (yaml_ok && root_node["max_files"])
            {
              log_max_files = root_node["max_files"].as<std::size_t>();
            }
        }
      else
        {
          log_max_files = std::atoi(std::getenv("H5_BRIDGE_LOG_MAX_FILES"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::size_t log_queue_size = 8192;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_ASYNC_QUEUE_SIZE") == nullptr)
        {
          if (yaml_ok && root_node["queue_size"])
            {
              log_queue_size = root_node["queue_size"].as<std::size_t>();
            }
        }
      else
        {
          log_queue_size =
            std::atoi(std::getenv("H5_BRIDGE_LOG_ASYNC_QUEUE_SIZE"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::size_t log_nthreads = 1;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_ASYNC_QUEUE_NTHREADS") == nullptr)
        {
          if (yaml_ok && root_node["n_threads"])
            {
              log_nthreads = root_node["n_threads"].as<std::size_t>();
            }
        }
      else
        {
          log_nthreads =
            std::atoi(std::getenv("H5_BRIDGE_LOG_ASYNC_QUEUE_NTHREADS"));
        }
    }
  catch (const std::exception& ex)
    { }

  std::string format = H5_BRIDGE_LOGGER_FORMAT;
  try
    {
      if (std::getenv("H5_BRIDGE_LOG_FORMAT") == nullptr)
        {
          if (yaml_ok && root_node["logger_format"])
            {
              format = root_node["logger_format"].as<std::string>();
            }
        }
      else
        {
          format = std::string(std::getenv("H5_BRIDGE_LOG_FORMAT"));
        }
    }
  catch (const std::exception& ex)
    { }

  h5_bridge::Logging::Init(log_level,
                           log_flush_level,
                           format,
                           log_dir,
                           log_file,
                           log_max_file_size,
                           log_max_files,
                           log_queue_size,
                           log_nthreads);
}
