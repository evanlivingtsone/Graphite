#pragma once

/** \file logger.h
Implements simple minimalist single-header cross-platform thread-safe logging facility using not-buffered file writing
logs to specified file and to console (if present)
macros LOG (LOG_ERROR, LOG_DEBUG) logs function name where logging occurred
Output format: HH:MM:SS:FFF thread_id log_category: optional_function_name: log-message
To suppress logging define LOGGING_DISABLED before this header inclusion

usage example:
/////////////////////////////////////////////////
#include "logger.h"

int main()
{
	logging::set_output("filename.log");
	logging::set_minimal_category(logging::cat_info);

	log_error("to log w/o logging place info, uses %s format", "printf()");
	LOG("to log with function name and with single %s", "argument");
	log_debug("debug message shouldn't appear because of category filter");

	logging::set_buffer_size(98);
	LOG("this long log message should be truncated because of too short buffer");

	return 0;
} 
/////////////////////////////////////////////////

resulting filename.log content:
/////////////////////////////////////////////////
08:30:20:609  3412 ERROR: to log w/o logging place info, uses printf() format
08:30:20:609  3412 info: main: to log with function name and with single argument
08:30:20:625  3412 info: main: this long log message should be truncated because of too short buf
/////////////////////////////////////////////////

Copyright Andriy Tylychko 
no restrictions on use

version 2.0
*/

#if !defined(LOGGING_DISABLED)

#include <stdio.h>
#include <stdarg.h>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/interprocess/detail/os_thread_functions.hpp> // for native thread id
#include <boost/detail/interlocked.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
namespace pt = boost::posix_time;
#include <boost/lexical_cast.hpp>
#include <exception>

#if defined(WIN32)
#	if !defined(snprintf)
#		define snprintf _snprintf
#		define vsnprintf _vsnprintf
#	endif
#endif

namespace logging
{
	/** logging category, or "priority" in some libs, allows to make some log-messages more important then others */
	enum Category {cat_error, cat_info, cat_debug};

	namespace details
	{
		/** workaround for compilers that cannot correctly instantiate local static variable if function is inlined */
		template <typename T>
		struct Inline_static_workaround
		{
			T& get()
			{
				static T t;
				return t;
			}
		};

		/** logging implementation */
		class Logger
		{
		public:
			explicit Logger(): minimal_category_(cat_debug), buf_size_(4096)
			{}

			static Logger& singleton()
			{
				static Inline_static_workaround<Logger> logger;
				//static Logger logger;
				return logger.get();
			}

			// not thread-safe operations so it's recommended to set log output only once before any logging
			void set_output(char const* filename)
			{
				if (ofs_.is_open())
					ofs_.close();
				ofs_.open(filename);
				if (!ofs_)
					throw std::runtime_error((std::string("cannot create log file: ") + filename).c_str());
				ofs_.rdbuf()->pubsetbuf(0, 0); // to suppress buffering
			}

			void set_minimal_category(Category cat)
			{
				BOOST_INTERLOCKED_EXCHANGE(&minimal_category_, long(cat));
			}

			void set_buffer_size(size_t buf_size)
			{
				BOOST_INTERLOCKED_EXCHANGE(&buf_size_, buf_size);
			}

			void log(Category category, char const* function_name, char const* format, va_list va)
			{	// lock-free logging implementation (except locking inside std::ofstream)
				if (category > minimal_category_)
					return;

				if (!buffer_.get())
					buffer_.reset(new std::string(buf_size_, '\0')); // allocate thread-local buffer

				// format log message
				std::string& buf = *buffer_;
				buf.resize(buf_size_);
				char* cur_buf = &buf[0];
				size_t size = buf.size() - 1; // for trailing \n
				pt::time_duration td(pt::microsec_clock::universal_time().time_of_day());
				size_t millisecs = (size_t)td.fractional_seconds() / 1000;
				static char const* const category_str[] = {"ERROR", "info", "debug"};
				int written;
				if (function_name) {
					written = snprintf(cur_buf, size, "%02i:%02i:%02i:%03i % 5i %s: %s: ", td.hours(), td.minutes(), td.seconds(), 
						millisecs, boost::interprocess::detail::get_current_thread_id(), category_str[category], function_name);
				} else {
					written = snprintf(cur_buf, size, "%02i:%02i:%02i:%03i % 5i %s: ", td.hours(), td.minutes(), td.seconds(), 
						millisecs, boost::interprocess::detail::get_current_thread_id(), category_str[category]);
				}
				cur_buf += written;
				size -= written;
				written = vsnprintf(cur_buf, size, format, va);
				if (written != -1 && written < (int)size)
					written += buf.size() - size - 1;
				else
					written = buf.size() - 1;
				buf[written] = '\n';
				buf.resize(written + 1);

				if (ofs_)
					ofs_.write(&buf[0], buf.size()); // thread safe operation
				std::cout << buf;
			}

		private:
			long volatile minimal_category_; // long instead of Category for interlocked exchange
			long volatile buf_size_;
			boost::thread_specific_ptr<std::string> buffer_;
			std::ofstream ofs_;
		};

		/** trick to be able to implement LOG macro that logs function name and accepts "ellipsis" parameters */
		struct Function_name_log_helper
		{
			Function_name_log_helper(const char* func_name, Category cat): func_name_(func_name), cat_(cat)
			{}

			void log(char const* format, ...) 
			{
				va_list va;
				va_start(va, format);
				Logger::singleton().log(cat_, func_name_, format, va);
				va_end(va);
			}

		private:
			char const* func_name_;
			Category cat_;
		};
	}

	/** opens given file for logging output, file writing is not buffered so even on app crash you'll have all log-messages stored in log-file */
	inline void set_output(char const* filename)
	{
		details::Logger::singleton().set_output(filename);
	}

	/** sets filter for log-messages, only messages more important than given "minimal category" will be logged */
	inline void set_minimal_category(Category cat)
	{
		details::Logger::singleton().set_minimal_category(cat);
	}

	/** sets buffer size for log-message formatting, by default it's 4096 chars */
	inline void set_buffer_size(size_t buf_size)
	{
		details::Logger::singleton().set_buffer_size(buf_size);
	}
}

/** logs error messages */
inline void log_error(char const* format, ...)
{ 
	va_list va;
	va_start(va, format);
	logging::details::Logger::singleton().log(logging::cat_error, NULL, format, va);
	va_end(va);
}

/** logs regular info messages */
inline void log_info(char const* format, ...)
{ 
	va_list va;
	va_start(va, format);
	logging::details::Logger::singleton().log(logging::cat_info, NULL, format, va);
	va_end(va);
}

/** just an alias for log_info() */
inline void log(char const* format, ...)
{ 
	va_list va;
	va_start(va, format);
	logging::details::Logger::singleton().log(logging::cat_info, NULL, format, va);
	va_end(va);
}

/** logs debug messages, this category is recommended for messages that is required only for deep diagnosis and usually is turned off by logging::set_minimal_category() */
inline void log_debug(char const* format, ...)
{ 
	va_list va;
	va_start(va, format);
	logging::details::Logger::singleton().log(logging::cat_debug, NULL, format, va);
	va_end(va);
}

/** following macros allows automatic logging of function name where logging occurred */
#	define MLOG logging::details::Function_name_log_helper(__FUNCTION__, logging::cat_info).log
#	define MLOG_ERROR logging::details::Function_name_log_helper(__FUNCTION__, logging::cat_error).log
#	define MLOG_DEBUG logging::details::Function_name_log_helper(__FUNCTION__, logging::cat_debug).log

#else // LOGGING_DISABLED
namespace logging
{
	enum Category {cat_error, cat_info, cat_debug};

	inline void set_output(char const* filename) {}
	inline void set_minimal_category(Category cat) {}
	inline void set_buffer_size(size_t buf_size) {}
}

inline void log_error(char const* format, ...) {}
inline void log_info(char const* format, ...) { }
inline void log(char const* format, ...) {}
inline void log_debug(char const* format, ...) {}

#	define MLOG
#	define MLOG_ERROR
#	define MLOG_DEBUG

#endif // LOGGING_DISABLED


