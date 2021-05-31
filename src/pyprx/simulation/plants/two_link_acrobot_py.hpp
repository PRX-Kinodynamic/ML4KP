#include <iostream>
#include <boost/python.hpp>

#include "prx/simulation/plants/two_link_acrobot.hpp"
// #include "prx/simulation/plant.hpp"
 
using namespace boost::python;

void pyprx_simulation_plants_acrobot_py()
{

   	class_<prx::two_link_acrobot_t, std::shared_ptr<prx::two_link_acrobot_t>, bases<prx::system_t>>("two_link_acrobot", no_init)
            // ("two_link_acrobot_t", init<std::string>())
        .def("__init__", make_constructor(&create_system_ptr<prx::two_link_acrobot_t>, default_call_policies(), (arg("path"))))
   	    // .def("set_state", &prx::two_link_acrobot_t::set_state)
   	    // .def("get_state", &prx::two_link_acrobot_t::get_state)
   	    // .def("set_control", &prx::two_link_acrobot_t::set_control)
        // .def("get_control", &prx::two_link_acrobot_t::get_control)
   	    // .def("to_string", &prx::two_link_acrobot_t::to_string)
        .def("propagate", &prx::two_link_acrobot_t::propagate)
   	    .def("update_configuration", &prx::two_link_acrobot_t::update_configuration)
   	    // .def("apply_lqr", &prx::two_link_acrobot_t::apply_lqr)
   	    // .def("get_control_space", &prx::two_link_acrobot_t::get_control_space, return_internal_reference<>())
        // .def("to_ptr", &create_ptr<prx::two_link_acrobot_t, prx::system_t> )
   	    ;
}
