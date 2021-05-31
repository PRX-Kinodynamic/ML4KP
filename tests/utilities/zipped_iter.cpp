#define BOOST_AUTO_TEST_MAIN zipped_iter_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/zipped_iter.hpp"


BOOST_AUTO_TEST_CASE( zipped_iter_test )
{  
	std::vector<int> integers = {0,1,2,3,4,5,6,7,8,9,10};
	std::vector<std::string> names = {"zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten"};
	std::vector<double> doubles = {0.1,1.1,2.1,3.1,4.1,5.1,6.1,7.1,8.1,9.1,10.1};

	std::string str;
	int num;
	double fl;
	// for (auto t : prx::zipped_t(integers.begin(), integers.end(), names.begin(), names.end()))
	for (auto t : prx::zip_iters(integers, names, doubles) )
	{
		std::tie(num, str, fl) = prx::unzip(t);
		// std::cout << *(std::get<1>(t)) << ": " << *(std::get<0>(t)) << std::endl;
		// std::cout << str << ": " << num << " " << fl << std::endl;
		// std::cout << names[num] << " == " << str << std::endl;
    	BOOST_CHECK(names[num] == str);
    	BOOST_CHECK(num == std::floor(fl));

	}
	
}