#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/basic_geoms/cylinder.hpp"

using namespace boost::python;

// std::shared_ptr<prx::box_t> create_box(std::string object_name, double dim_x, double dim_y, double dim_z, const prx::transform_t pose)
// {
// 	return std::make_shared<prx::box_t>(object_name, dim_x, dim_y, dim_z, pose);
// }

void pyprx_utilities_geometry_basic_geoms_cylinder_py()
{

   	// class_<prx::box_t, std::shared_ptr<prx::box_t>, bases<prx::movable_object_t>>("box", no_init)
   	// 	.def("create_obstacle", &create_box).staticmethod("create_obstacle");
   	// 	;

   	// implicitly_convertible<std::shared_ptr<prx::box_t>, std::shared_ptr<prx::movable_object_t>>();
}