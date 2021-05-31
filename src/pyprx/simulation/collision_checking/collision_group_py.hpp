#include "prx/simulation/collision_checking/collision_group.hpp"

void pyprx_simulation_collision_checking_collision_group_py()
{

	// register_ptr_to_python< std::shared_ptr<prx::collision_group_t> >();
	class_<std::shared_ptr<prx::collision_group_t>>("collision_group")
		.def("__init__", make_constructor(&init_as_ptr<prx::collision_group_t, std::vector<prx::system_ptr_t>, std::vector<std::shared_ptr<prx::movable_object_t>>>, default_call_policies(), (args("in_plants"), args("in_obstacles")) ))
		// .def("create_ptr", &create_ptr<prx::collision_group_t, prx::collision_group_t> )
		;
}