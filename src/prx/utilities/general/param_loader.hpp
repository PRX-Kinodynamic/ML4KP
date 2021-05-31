#pragma once

#include <regex>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/prx_assert.hpp"

namespace prx
{
	class param_loader
	{
	public:

		// typedef std::unordered_map<std::string, std::string>::iterator iterator;
		// typedef std::unordered_map<std::string, std::string>::const_iterator const_iterator;
		typedef YAML::Node::iterator iterator;
		typedef YAML::Node::const_iterator const_iterator;

		param_loader();
		param_loader(std::string file_name);
		param_loader(int argc, char* argv[]);
		param_loader(std::string file_name, int argc, char* argv[]);
		param_loader(const param_loader& pl);

		void add_file(std::string file_name);

		void add_opts(int argc, char* argv[]);

		inline const std::string get_input_path() const
		{
			return pl_input_path;
		}

		inline const void set_input_path(const std::string& new_path)
		{
			pl_input_path = new_path;
			if (pl_input_path.back() != '/')
			{
				pl_input_path += "/";
			}
		}

		const param_loader operator[](const std::string& key) const;

		param_loader operator[](const std::string& key);

		void add(const param_loader& pl);

		template<typename T>
		void set(T val){params = val;}
		void print_variables();

		void print();

		inline bool exists(const std::string& key)
		{
			return !params[key].IsNull();
		}

		template<typename T=std::string>
		T as()
		{
			try
			{
				return params.as<T>();
			}
			catch(...)
			{
				std::cout << params << std::endl;
				prx_throw_backtrace("Tried to convert to an incorrect type for: ");
			}
		}

		template <typename T>
  		param_loader& operator=(const T& rhs)
  		{
  			this -> params = rhs;
  			return *this;
  		}

		inline iterator begin()
		{
			return params.begin();
		}

		inline iterator end()
		{
			return params.end();
		}

		inline const_iterator begin() const
		{
			return params.begin();
		}

		inline const_iterator end() const
		{
			return params.end();
		}

	protected:

		param_loader(YAML::Node input_params);

		YAML::Node expand_file(YAML::Node& node);

		YAML::Node find(const std::string& key, YAML::Node node);

		void print(const YAML::Node& pl, std::string prepath = "");

		YAML::Node params;

		void merge(const YAML::Node& other);

		// std::unordered_map<std::string, param_loader> params;
		std::string pl_input_path;

	};
}