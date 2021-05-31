#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/system.hpp"


struct system_wrap : prx::system_t, wrapper<prx::system_t>
{
	system_wrap(const std::string& path) : prx::system_t(path){};

    // int default_system_wrap(const std::string& path) { return this -> system_wrap::system_t(path); }

    void propagate(const double simulation_step)
    {
        this->get_override("propagate")(simulation_step);
    }
    void propagate_2(const double simulation_step, const prx::propagate_step step)
    {
        this->get_override("propagate")(simulation_step, step);
    }
    void add_system(prx::system_ptr_t& sp)
    {
        this->get_override("add_system")(sp);
		// this -> add_system(sp );
    }
    void compute_control()
    {
        this->get_override("compute_control")();
		// this -> compute_control();
    }
    void set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
    {
        this->get_override("set_state_space_bounds")();
		// this -> compute_control();
    }

    prx::space_t*   get_system_wrap_state_space(){return prx::system_t::state_space;}
	void 	  		set_system_wrap_state_space(prx::space_t* val){prx::system_t::state_space = val;}

	prx::space_t*   get_system_wrap_input_control_space(){return prx::system_t::input_control_space;}
	void 	   		set_system_wrap_input_control_space(prx::space_t* val){prx::system_t::input_control_space = val;}

};

void set_simulation_step(double ss)
{
	prx::simulation_step = ss;
}

void pyprx_simulation_system_py()
{

	scope().attr("simulation_step") = prx::simulation_step;
	def("set_simulation_step", &set_simulation_step);

	class_<system_wrap, boost::noncopyable>("system", init<std::string>())
        // .def("__init__", make_constructor(&init_as_ptr<system_wrap, std::string>, default_call_policies()))
		// .def("__init__", make_constructor(&system_wrap, default_call_policies(), (arg("path")) ))
        // .def("__init__", make_constructor(&fromAxisAngle, default_call_policies(),(arg("axis"),  arg("angle"))))
		.def("get_state_space", &prx::system_t::get_state_space, return_internal_reference<>())
		.def("get_control_space", &prx::system_t::get_state_space, return_internal_reference<>())
	 	.def("add_system", pure_virtual(&prx::system_t::add_system))
	 	.def("propagate", pure_virtual(&system_wrap::propagate))
	 	.def("propagate", &system_wrap::propagate_2)
	 	.def("compute_control", pure_virtual(&prx::system_t::compute_control))
	 	.def("compute_stopping_maneuver", &prx::system_t::compute_stopping_maneuver)
	 	.def("finalize_system_tree", &prx::system_t::finalize_system_tree)
	 	.def("set_state_space_bounds", pure_virtual(&prx::system_t::set_state_space_bounds))
	 	.def("get_pathname", &prx::system_t::get_pathname)
	 	.def("get_system_type", &prx::system_t::get_system_type)
      	.def("get_state_space", &system_wrap::get_system_wrap_state_space, return_internal_reference<>())
      	.def("set_state_space", &system_wrap::set_system_wrap_state_space, return_internal_reference<>())
      	.def("get_input_control_space", &system_wrap::get_system_wrap_input_control_space, return_internal_reference<>())
      	.def("set_input_control_space", &system_wrap::set_system_wrap_input_control_space, return_internal_reference<>())
	 	// .def("", &prx::system_t::)
	 	// .def("", &prx::system_t::)
	 	// .def("", &prx::system_t::)
		// virtual inline const space_t* get_state_space() const
		// .def("create_ptr", &create_ptr<prx::system_t, prx::system_t> ).staticmethod("create_ptr")
		;
	register_ptr_to_python< prx::system_ptr_t >();

	class_<std::vector<prx::system_ptr_t>>("vector_system")
   		.def(vector_indexing_suite<std::vector<prx::system_ptr_t>>())
		;

}