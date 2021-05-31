#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/geometry.hpp"
#include "pyprx/utilities/geometry/movable_object_py.hpp"
#include "pyprx/utilities/geometry/basic_geoms/basic_geoms_py.hpp"

using namespace boost::python;
void pyprx_utilities_geometry_py()
{

   	class_<prx::geometry_t>("geometry", init<prx::geometry_type_t>())
   	    ;

   	pyprx_utilities_geometry_movable_object_py();
   	pyprx_utilities_geometry_basic_geoms_py();

   	// register_ptr_to_python<std::shared_ptr<prx::movable_object_t>>();

}