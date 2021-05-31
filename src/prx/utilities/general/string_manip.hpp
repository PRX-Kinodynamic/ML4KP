#pragma once
/**
 * @file string_manip.hpp
 * @author Zakary Littlefield
 * @brief <b> Some helper functions for string manipulation. </b>
 * */
#include <utility>
#include <string>

namespace prx
{
	/**
	 * Splits a /-delimited path into the remainder and the beginning
	 * 
	 * For example: 
	 * 
	 * path = "a/b/c/d"
	 * split = split_path(path);
	 * split.first = "a"
	 * split.second = "b/c/d"
	 * 
	 * 
	 * @brief Splits a pathname based on slashes "/"
	 * @param path The path to be split
	 * @return A pair containing the split path and the remainder
	 */
	std::pair<const std::string, const std::string> split_path(const std::string& path, char delimiter = '/');

	/**
	 * Splits a /-delimited path into the first part and the tail
	 * 
	 * For example: 
	 * 
	 * path = "a/b/c/d"
	 * split = reverse_split_path(path);
	 * split.first = "a/b/c"
	 * split.second = "d"
	 * 
	 * 
	 * @brief Splits a pathname based on slashes "/"
	 * @param path The path to be split
	 * @return A pair containing the split path and the remainder
	 */
	std::pair<const std::string, const std::string> reverse_split_path(const std::string& path, char delimiter = '/');

	/**
	 *  Performs string comparison between two strings. The difference 
	 *  is that comparison is done from the end of the string to the beginning.
	 *  This can be especially helpful for system trees, where only the last few characters differ.
	 * @brief Compare two strings in reverse order.
	 * @param str1 The first string to compare.
	 * @param str2 The second string to compare.
	 * @return True if the strings are identical.
	 */
	bool reverse_string_compare(const std::string& str1,const std::string& str2);

	/**
	 * Replaces the first occurrence of a substring
	 * inside a string with another substring.
	 * @brief A string replacement helper function. 
	 * @param input Input string containing the substring to be replaced.
	 * @param to_replace The substring whose first occurrence needs to be replaced.
	 * @param replace_with The substring that it is replaced with.
	 */
	std::string replace_first(std::string &input, const std::string& to_replace, const std::string& replace_with);

	/**
	 * @brief Test if one string is the prefix of an input string.
	 * @param test_string The string that needs to be checked if it is a prefix.
	 * @param input The input string which needs to be checked.
	 */
	bool is_prefix(const std::string& test_string, const std::string& input);

	/**
	 *	@brief Test if a string is a substring of an input string.
	 *  @param full_string The full string that needs to be checked for substring existence.
	 *  @param sub_str The substring that needs to be checked.
	 */
	bool is_subset(const std::string& sub_str, const std::string& full_string);

	/**
	 * @brief A helper function for displaying a progress bar on the command line.
	 * 
	 *  Fixed width progress bar given value between [0,1]. For e.g., if 10 iterations
	 *  out of 40 are complete, the progress bar will be 25% full if the appropriate input
	 * (0.25) is passed.
	 * 
	 * @param value A value between [0,1] that indicates the fraction of the progress that 
	 * has been measured.
	 */

	void output_progress_bar(double value);

}