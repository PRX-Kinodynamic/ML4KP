#include "prx/simulation/playback/plan.hpp"

const prx::plan_step_t& (prx::plan_t::*plan_at_1)(unsigned)const  = &prx::plan_t::at;
prx::space_point_t (prx::plan_t::*plan_at_2)(double)  const  = &prx::plan_t::at;

// void (prx::trajectory_t::*copy_onto_back_1)(prx::space_point_t)  = &prx::trajectory_t::copy_onto_back;
// void (prx::trajectory_t::*copy_onto_back_2)(const prx::space_t*) = &prx::trajectory_t::copy_onto_back;

void pyprx_simulation_playback_plan_py()
{

	class_<prx::plan_step_t, std::shared_ptr<prx::plan_step_t>>("plan_step", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::plan_step_t, prx::space_point_t, double>, default_call_policies(), (args("space")) ))
		.def("copy_step", &prx::plan_step_t::copy_step)
		.add_property("control", &prx::plan_step_t::control)
		.add_property("duration", &prx::plan_step_t::duration)
		;

	class_<prx::plan_t, std::shared_ptr<prx::plan_t>>("plan", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::plan_t, prx::space_t*>, default_call_policies(), (args("new_space")) ))
		.def("__init__", make_constructor(&init_as_ptr<prx::plan_t, prx::plan_t>, default_call_policies(), (args("other")) ))
		.def("__len__", &prx::plan_t::size)
		.def("size", &prx::plan_t::size)
		.def("duration", &prx::plan_t::duration)
		// .def("__getitem__", plan_at_1)
		// .def("__getitem__", plan_at_2)
		// .def("front", &prx::plan_t::front)
		// .def("back", &prx::plan_t::back)
		.def("__iter__", iterator<prx::plan_t>())
		.def("resize", &prx::plan_t::resize)
		.def(self += other<prx::plan_t>())
		.def("clear", &prx::plan_t::clear)
		.def("copy_to", &prx::plan_t::copy_to)
		.def("copy_onto_back", &prx::plan_t::copy_onto_back)
		.def("copy_onto_front", &prx::plan_t::copy_onto_front)
		.def("append_onto_front", &prx::plan_t::append_onto_front)
		.def("append_onto_back", &prx::plan_t::append_onto_back)
		.def("extend_last_control", &prx::plan_t::extend_last_control)
		.def("pop_front", &prx::plan_t::pop_front)
		.def("pop_back", &prx::plan_t::pop_back)
		.def("print", &prx::plan_t::print)

		;
}