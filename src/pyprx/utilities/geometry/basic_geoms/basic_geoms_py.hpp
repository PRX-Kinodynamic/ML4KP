#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/geometry/basic_geoms/box_py.hpp"
#include "pyprx/utilities/geometry/basic_geoms/cylinder_py.hpp"
#include "pyprx/utilities/geometry/basic_geoms/sphere_py.hpp"

using namespace boost::python;
void pyprx_utilities_geometry_basic_geoms_py()
{

	pyprx_utilities_geometry_basic_geoms_box_py();
	pyprx_utilities_geometry_basic_geoms_cylinder_py();
	pyprx_utilities_geometry_basic_geoms_sphere_py();
}