#pragma once

#if defined(PROJECT_LOG_BACKEND_SPDLOG) && PROJECT_LOG_BACKEND_SPDLOG

#include <spdlog/spdlog.h>

#define PROJECT_LOG_DEBUG(...) spdlog::debug(__VA_ARGS__)
#define PROJECT_LOG_INFO(...) spdlog::info(__VA_ARGS__)
#define PROJECT_LOG_WARN(...) spdlog::warn(__VA_ARGS__)
#define PROJECT_LOG_ERROR(...) spdlog::error(__VA_ARGS__)
#define PROJECT_LOG_CRITICAL(...) spdlog::critical(__VA_ARGS__)
#define PROJECT_LOG_SHOULD_LOG_DEBUG() spdlog::should_log(spdlog::level::debug)
#define PROJECT_LOG_HAS_DEFAULT_LOGGER() (spdlog::default_logger_raw() != nullptr)
#define PROJECT_LOG_FLUSH_DEFAULT()                \
  do {                                             \
    if (spdlog::default_logger_raw() != nullptr) { \
      spdlog::default_logger_raw()->flush();       \
    }                                              \
  } while (false)

#else

#define PROJECT_LOG_DEBUG(...) ((void)0)
#define PROJECT_LOG_INFO(...) ((void)0)
#define PROJECT_LOG_WARN(...) ((void)0)
#define PROJECT_LOG_ERROR(...) ((void)0)
#define PROJECT_LOG_CRITICAL(...) ((void)0)
#define PROJECT_LOG_SHOULD_LOG_DEBUG() (false)
#define PROJECT_LOG_HAS_DEFAULT_LOGGER() (false)
#define PROJECT_LOG_FLUSH_DEFAULT() ((void)0)

#endif
