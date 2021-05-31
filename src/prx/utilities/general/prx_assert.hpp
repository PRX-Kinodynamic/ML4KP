#pragma once

#include <exception>
#include <string>
#include <sstream>
#include <unistd.h>
#include <iostream>
#include <execinfo.h>

namespace prx
{
	#define COLOR_NORMAL "\033[0m"
  	#define COLOR_RED "\033[31m"
  	#define COLOR_GREEN "\033[32m"
  	#define COLOR_YELLOW "\033[33m"
	class prx_assert_t : public std::exception
	{
	private:
		const char* expression;
		const char* file;
		int line;
		std::string message;
		std::string report;

	public:

		class stream_t
		{
			std::ostringstream stream;

		public:
			operator std::string() const
			{
				return stream.str();
			}

			template<typename T>
			stream_t& operator << (const T& value)
			{
				stream << value;
				return *this;
			}
		};

		void logger()
		{
			std::cerr << report << std::endl;
		}

		prx_assert_t(const char* expression, const char* file, int line, const std::string& message)
			: expression(expression), file(file), line(line), message(message)
		{
			std::ostringstream out_stream;

			if (!message.empty())
			{
				out_stream << message << ": ";
			}

			out_stream << "Assertion '" << expression << "'";
			out_stream << " failed in file '" << file << "' line " << line;
			report = out_stream.str();
			logger();
		}

		prx_assert_t(const std::string& message)
			: message(message)
		{
			std::ostringstream out_stream;

			if (!message.empty())
			{
				out_stream << message << ": ";
			}
			report = out_stream.str();
		}

		prx_assert_t()
		{
		}

		prx_assert_t get_backtrace(std::string message = "", size_t size = 10)
		{
			char **strings;
			void *buffer[size];
			std::ostringstream out_stream;

			// get void*'s for all entries on the stack
			int bt_size = backtrace(buffer, size);

			strings = backtrace_symbols(buffer, bt_size);
			if (!message.empty())
			{
				out_stream << message << ": " << std::endl;
			}

			for (int i = 0; i < bt_size; ++i)
			{
				// TODO: Print the line number
				out_stream << strings[i] << std::endl;
			}
			report = out_stream.str();
			return *this;
		}

		virtual const char* what() const throw()
		{
			return report.c_str();
		}

		const char* get_expression() const throw()
		{
			return expression;
		}

		const char* get_file() const throw()
		{
			return file;
		}

		int get_line() const throw()
		{
			return line;
		}

		const char* get_message() const throw()
		{
			return message.c_str();
		}

		~prx_assert_t() throw()
		{
		}
	};


	#define prx_assert(EXPRESSION, MESSAGE) if(!(EXPRESSION)) { throw prx::prx_assert_t(#EXPRESSION, __FILE__, __LINE__, (prx::prx_assert_t::stream_t() << MESSAGE)); }
	#define prx_throw(MESSAGE) {throw prx::prx_assert_t("", __FILE__, __LINE__, (prx::prx_assert_t::stream_t() << MESSAGE)); }
	#define prx_throw_quiet(MESSAGE) {throw prx::prx_assert_t((prx::prx_assert_t::stream_t() << MESSAGE)); }

	/**
	 * throw error printing the bactrace
	 * @param  __VA_ARGS__ At most two arguments: First the message, second the max size of the stack to print.
	 * @return             Throws the error printing the backtrace
	 */
	#define prx_throw_backtrace(...) {throw prx::prx_assert_t().get_backtrace(__VA_ARGS__); }

	#define prx_warn(MESSAGE) std::cerr << COLOR_YELLOW << "[PRX WARN] " << __PRETTY_FUNCTION__ << ":" << __LINE__ << " " << MESSAGE << COLOR_NORMAL << std::endl;
	#define prx_warn_cond(EXPRESSION, MESSAGE) if (!(EXPRESSION)) {prx_warn(#EXPRESSION << " " << MESSAGE)}
}