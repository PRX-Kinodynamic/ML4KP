#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/basic_geoms/box.hpp"
// #include "pyprx/utilities/geometry/movable_object_py.hpp"

using namespace boost::python;

// prx::box_t* create_box(std::string object_name, double dim_x, double dim_y, double dim_z, const prx::transform_t pose)
std::shared_ptr<prx::box_t> create_box(std::string object_name, double dim_x, double dim_y, double dim_z, const prx::transform_t pose)
{
	// prx::transform_t obstacle_pose0;
	// obstacle_pose0.setIdentity();
	// return new prx::box_t(object_name, dim_x, dim_y, dim_z, obstacle_pose0);
	// return new prx::box_t(object_name, dim_x, dim_y, dim_z, pose);
	return std::make_shared<prx::box_t>(object_name, dim_x, dim_y, dim_z, pose);
}

void pyprx_utilities_geometry_basic_geoms_box_py()
{

   	class_<prx::box_t, std::shared_ptr<prx::box_t>, bases<prx::movable_object_t>>("box", no_init)
   		.def("create_obstacle", &create_box).staticmethod("create_obstacle");
   			// init<std::string, double, double, double, prx::transform_t&>(), return_internal_reference<>())
   		// .def("create_box", init<std::string, double, double, double, prx::transform_t&>(), return_internal_reference<>())
   		// .def("create_box", &create_box, return_internal_reference<>())
   		// .def(init<std::string, double, double, double, prx::transform_t&, std::string>())
   		// .def("create_obstacle", &create_ptr<prx::box_t, prx::box_t>, with_custodian_and_ward_postcall<1, 0>())
   		// .def("create_obstacle",&create_ptr<prx::box_t, prx::movable_object_t>).staticmethod("create_obstacle")
   	    ;
   	// def("create_obstacle", &create_box, return_value_policy<manage_new_object>());

   	implicitly_convertible<std::shared_ptr<prx::box_t>,
                                 std::shared_ptr<prx::movable_object_t>>();
   	// def("create_obstacle", &create_ptr<prx::box_t, prx::movable_object_t>);
   	// register_ptr_to_python<std::shared_ptr<prx::box_t>>();

}