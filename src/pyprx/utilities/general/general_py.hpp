#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/general/constants_py.hpp"
#include "pyprx/utilities/general/transforms_py.hpp"
#include "pyprx/utilities/general/random_py.hpp"

void pyprx_utilities_general()
{
	pyprx_utilities_general_constants();
	pyprx_utilities_general_transforms();
	pyprx_utilities_general_random_py();
}