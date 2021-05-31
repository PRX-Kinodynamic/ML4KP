#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/general/constants.hpp"

using namespace boost::python;


void pyprx_utilities_general_constants()
{
	// Var("EPSILON", PRX_EPSILON);
	scope().attr("PRX_EPSILON") = PRX_EPSILON;
	scope().attr("PRX_INFINITY") = PRX_INFINITY;
	scope().attr("PRX_PI") = PRX_PI;

}