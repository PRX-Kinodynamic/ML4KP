#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/playback/trajectory.hpp"


prx::space_point_t (prx::trajectory_t::*at_1)(unsigned)const  = &prx::trajectory_t::at;
prx::space_point_t (prx::trajectory_t::*at_2)(double)  const  = &prx::trajectory_t::at;

void (prx::trajectory_t::*copy_onto_back_1)(prx::space_point_t)  = &prx::trajectory_t::copy_onto_back;
void (prx::trajectory_t::*copy_onto_back_2)(const prx::space_t*) = &prx::trajectory_t::copy_onto_back;

void pyprx_simulation_playback_trajectory_py()
{

	class_<prx::trajectory_t, std::shared_ptr<prx::trajectory_t>>("trajectory", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::trajectory_t, prx::space_t*>, default_call_policies(), (args("space")) ))
		.def("__init__", make_constructor(&init_as_ptr<prx::trajectory_t, prx::trajectory_t>, default_call_policies(), (args("traj")) ))
		.def("__len__", &prx::trajectory_t::size)
		.def("__getitem__", at_1)
		.def("__getitem__", at_2)
		.def("__iter__", iterator<prx::trajectory_t>())
		.def("front", &prx::trajectory_t::front)
		.def("back", &prx::trajectory_t::back)
		.def("copy_onto_back", copy_onto_back_1)
		// .def("copy_onto_back", copy_onto_back_2)
		.def(self += other<prx::trajectory_t>())
		.def(self == other<prx::trajectory_t>())
		.def(self != other<prx::trajectory_t>())

		;
}