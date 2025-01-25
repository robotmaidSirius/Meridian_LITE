/**
 * @file i_mrd_diagnostic.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */
#ifndef I_MRD_COM_DIAGNOSTIC_HPP
#define I_MRD_COM_DIAGNOSTIC_HPP

#include "Meridim90.hpp"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

namespace meridian {
namespace core {
namespace communication {
using namespace meridian::core::meridim;

class IMeridianDiagnostic {
public:
  // @brief Define log level
  typedef enum output_log_level_t {
    OUTPUT_LOG_LEVEL_ALL,     /*!< Any logging levels that have been configured are logged at this log level. */
    OUTPUT_LOG_LEVEL_TRACE,   /*!< The TRACE log level records all of the application's behaviour details. Its purpose is primarily diagnostic, and it is more granular and finer than the DEBUG log level. */
    OUTPUT_LOG_LEVEL_DEBUG,   /*!< You are providing diagnostic information in a thorough manner with DEBUG. It's long and contains more information than you'll need when using the application. */
    OUTPUT_LOG_LEVEL_INFO,    /*!< INFO messages are similar to how applications normally behave. */
    OUTPUT_LOG_LEVEL_MESSAGE, /*!< When This log level signals operational messages. */
    OUTPUT_LOG_LEVEL_WARN,    /*!< When an unexpected application issue has been identified, the WARN log level is used.  This indicates that you are unsure if the issue will recur or not. At this time, you may not notice any negative effects on your application. */
    OUTPUT_LOG_LEVEL_ERROR,   /*!< This log level is used when a serious issue is preventing the application's functionalities from functioning properly. */
    OUTPUT_LOG_LEVEL_FATAL,   /*!< The FATAL level of logging indicates that the application's situation is critical, such as when a critical function fails. */
    OUTPUT_LOG_LEVEL_OFF      /*!< Nothing is logged at this level of logging. */
  } OUTPUT_LOG_LEVEL;

public:
  virtual bool setup() = 0;

protected:
  virtual size_t message(OUTPUT_LOG_LEVEL level, const char *message) = 0;

public:
  const char *get_text_level(OUTPUT_LOG_LEVEL level) {
    switch (level) {
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ALL:
      return "---";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE:
      return "TRA";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG:
      return "DEB";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO:
      return "INF";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE:
      return "MES";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN:
      return "WAR";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ERROR:
      return "ERR";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL:
      return "FAT";
    case OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_OFF:
      return "OFF";
    default:
      return "UNK";
    }
  }
  void set_level(OUTPUT_LOG_LEVEL level) { this->_level = level; }
  void enable() { this->_output_log = true; }
  void disable() { this->_output_log = false; }

  void log_trace(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_TRACE, format, arg);
    va_end(arg);
  }
  void log_debug(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_DEBUG, format, arg);
    va_end(arg);
  }
  void log_info(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO, format, arg);
    va_end(arg);
  }
  void log_message(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_MESSAGE, format, arg);
    va_end(arg);
  }
  void log_warn(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_WARN, format, arg);
    va_end(arg);
  }
  void log_error(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_ERROR, format, arg);
    va_end(arg);
  }
  void log_fatal(const char *format, ...) {
    va_list arg;
    va_start(arg, format);
    log_printf(OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_FATAL, format, arg);
    va_end(arg);
  }
  size_t log_printf(OUTPUT_LOG_LEVEL level, const char *format, ...) {
    if (true == this->_output_log) {
      if (this->_level <= level) {
        char loc_buf[64];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return 0;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return 0;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(level, message);
        if (message != loc_buf) {
          free(message);
        }
        return len;
      }
    }
    return 0;
  }

private:
  bool _output_log = true;                                           /*!< Output control flag   */
  OUTPUT_LOG_LEVEL _level = OUTPUT_LOG_LEVEL::OUTPUT_LOG_LEVEL_INFO; /*!< Output Level */
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // I_MRD_COM_DIAGNOSTIC_HPP
