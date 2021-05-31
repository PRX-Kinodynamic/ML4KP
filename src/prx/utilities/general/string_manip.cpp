#include "prx/utilities/general/string_manip.hpp"

#include <algorithm>
#include <iostream>

namespace prx
{
	std::pair<const std::string, const std::string> split_path(const std::string& path, char delimiter)
	{
		const size_t first_slash = path.find(delimiter);

		if( first_slash == path.npos )
			return std::make_pair(path, "");
		else
			return std::make_pair(
					path.substr(0, first_slash),
					path.substr(first_slash + 1, path.npos));
	}

	std::pair<const std::string, const std::string> reverse_split_path(const std::string& path, char delimiter)
	{
		const size_t first_slash = path.rfind(delimiter);

		if( first_slash == path.npos )
			return std::make_pair(path, "");
		else
			return std::make_pair(
					path.substr(0, first_slash),
					path.substr(first_slash + 1, path.npos));
	}
	
	bool reverse_string_compare(const std::string& str1,const std::string& str2)
	{
		return str1.size() == str2.size() && std::equal(str1.rbegin(), str1.rend(), str2.rbegin());
	}

	std::string replace_first(std::string &input, const std::string& to_replace, const std::string& replace_with)
	{
		return(input.replace(input.find(to_replace), to_replace.length(), replace_with));
	}

	bool is_prefix(const std::string& test_string, const std::string& comparison)
	{
		if(test_string.size()>comparison.size())
			return false;
		auto res = std::mismatch(test_string.begin(),test_string.end(),comparison.begin());
		return res.first==test_string.end() && *(res.second)=='/';
	}

	bool is_subset(const std::string& sub_str, const std::string& full_string)
	{
		return full_string.find(sub_str) != std::string::npos;
	}

	void output_progress_bar(double value)
	{
		int bar_width = 70;
		std::cout << "[";
		int pos = bar_width * value;
		for (int i = 0; i < bar_width; ++i) 
		{
			if (i < pos) 
				std::cout << "=";
			else if (i == pos) 
				std::cout << ">";
			else 
				std::cout << " ";
		}
		std::cout << "] " << int(value * 100.0) << " %\r";
		std::cout.flush();
	}
}