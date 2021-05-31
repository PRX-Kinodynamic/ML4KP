#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/spaces/space_py.hpp"
#include "pyprx/utilities/general/general_py.hpp"
#include "pyprx/utilities/geometry/geometry_py.hpp"
#include "pyprx/utilities/data_structures/data_structures_py.hpp"

void pyprx_utilities_py()
{
	pyprx_utilities_general();
	pyprx_utilities_geometry_py();
	pyprx_utilities_spaces_space();
	pyprx_utilities_data_structures_py();
}