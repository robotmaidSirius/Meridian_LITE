/**
 * @file i_mrd_diagnostic.hpp
 * @brief
 * @version 1.2.0
 * @date 2025-01-24
 *
 * @copyright Copyright (c) 2025-.
 *
 */
// 各モジュールから設定できるように、logのレベルを渡す引数を追加する
// log_xxxの中身が同じなので、コードを整理する
#ifndef __I_MRD_COM_DIAGNOSTIC_HPP__
#define __I_MRD_COM_DIAGNOSTIC_HPP__

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
  class Status {
  public:
    bool initalized = false;
    bool setup = false;
    bool happened_error = false;
    bool no_device = false;

  public:
    void all_ok() {
      this->initalized = true;
      this->setup = true;
      this->happened_error = false;
      this->no_device = true;
    }
  };

public:
  // @brief Define log level
  typedef enum output_log_level_t {
    LEVEL_ALL,         /*!< Any logging levels that have been configured are logged at this log level. */
    LEVEL_TRACE,       /*!< The TRACE log level records all of the application's behaviour details. Its purpose is primarily diagnostic, and it is more granular and finer than the DEBUG log level. */
    LEVEL_DEBUG,       /*!< You are providing diagnostic information in a thorough manner with DEBUG. It's long and contains more information than you'll need when using the application. */
    LEVEL_INFO,        /*!< INFO messages are similar to how applications normally behave. */
    LEVEL_WARN,        /*!< When an unexpected application issue has been identified, the WARN log level is used.  This indicates that you are unsure if the issue will recur or not. At this time, you may not notice any negative effects on your application. */
    LEVEL_ERROR,       /*!< This log level is used when a serious issue is preventing the application's functionalities from functioning properly. */
    LEVEL_FATAL,       /*!< The FATAL level of logging indicates that the application's situation is critical, such as when a critical function fails. */
    LEVEL_OPERATIONAL, /*!< When This log level signals operational messages. */
    LEVEL_OFF          /*!< Nothing is logged at this level of logging. */
  } OUTPUT_LOG_LEVEL;

public:
  virtual bool setup() { return false; }
  virtual const char *get_name() { return "None"; }

protected:
  virtual size_t message(OUTPUT_LOG_LEVEL level, const char *message) { return 0; }

public:
  const char *get_text_level(OUTPUT_LOG_LEVEL level) {
    switch (level) {
    case OUTPUT_LOG_LEVEL::LEVEL_ALL:
      return "---";
    case OUTPUT_LOG_LEVEL::LEVEL_TRACE:
      return "TRA";
    case OUTPUT_LOG_LEVEL::LEVEL_DEBUG:
      return "DEB";
    case OUTPUT_LOG_LEVEL::LEVEL_INFO:
      return "INF";
    case OUTPUT_LOG_LEVEL::LEVEL_WARN:
      return "WAR";
    case OUTPUT_LOG_LEVEL::LEVEL_ERROR:
      return "ERR";
    case OUTPUT_LOG_LEVEL::LEVEL_FATAL:
      return "FAT";
    case OUTPUT_LOG_LEVEL::LEVEL_OFF:
      return "OFF";
    case OUTPUT_LOG_LEVEL::LEVEL_OPERATIONAL:
      return "OPT";
    default:
      return "UNK";
    }
  }
  void set_level(OUTPUT_LOG_LEVEL level) { this->_level = level; }
  void enable() { this->_output_log = true; }
  void disable() { this->_output_log = false; }

  void log(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_OPERATIONAL;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  void log_trace(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_TRACE;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  void log_debug(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_DEBUG;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  void log_info(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_INFO;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  void log_warn(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_WARN;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  void log_error(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_ERROR;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  void log_fatal(const char *format, ...) {
    OUTPUT_LOG_LEVEL a_level = OUTPUT_LOG_LEVEL::LEVEL_FATAL;
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
        char *message = loc_buf;
        va_list arg;
        va_list copy;
        va_start(arg, format);
        va_copy(copy, arg);
        int len = vsnprintf(message, sizeof(loc_buf), format, copy);
        va_end(copy);
        if (len < 0) {
          va_end(arg);
          return;
        }
        if ((unsigned long long)len >= sizeof(loc_buf)) {
          message = (char *)malloc(len + 1);
          if (message == NULL) {
            va_end(arg);
            return;
          }
          len = vsnprintf(message, len + 1, format, arg);
        }
        va_end(arg);

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return;
      }
    }
    return;
  }
  size_t printf(OUTPUT_LOG_LEVEL a_level, const char *format, ...) {
    if (this->_output_log) {
      if (this->_level <= a_level) {
        char loc_buf[this->_BUFFER_SIZE];
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

        this->message(a_level, message);
        if (message != loc_buf) {
          free(message);
        }
        return len;
      }
    }
    return 0;
  }

private:
  bool _output_log = false;                               /*!< Output control flag   */
  OUTPUT_LOG_LEVEL _level = OUTPUT_LOG_LEVEL::LEVEL_INFO; /*!< Output Level */
  const int _BUFFER_SIZE = 128;

public:
  void get_status(Status &state) {
    state.initalized = this->a_state.initalized;
    state.setup = this->a_state.setup;
    state.happened_error = this->a_state.happened_error;
    state.no_device = this->a_state.no_device;
  }

protected:
  Status a_state;
};

} // namespace communication
} // namespace core
} // namespace meridian

#endif // __I_MRD_COM_DIAGNOSTIC_HPP__
