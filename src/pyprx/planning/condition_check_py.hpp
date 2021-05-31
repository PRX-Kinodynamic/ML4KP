#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include "prx/planning/condition_check.hpp"

using namespace boost::python;

void pyprx_planning_condition_check_py()
{

   	class_<prx::condition_check_t>("condition_check", no_init )//init<std::string, double>())   
   		.def("__init__", make_constructor(&init_as_ptr<prx::condition_check_t, std::string, double>, default_call_policies(), (args("type"), args("check"))) )
        .def("reset", &prx::condition_check_t::reset)
        .def("check", &prx::condition_check_t::check)
        .def("time", &prx::condition_check_t::time)
        .def("iterations", &prx::condition_check_t::iterations)
        // .def("condition", &prx::condition_check_t::condition)
   	    ;
   	
}