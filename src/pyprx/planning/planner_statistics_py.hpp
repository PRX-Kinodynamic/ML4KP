#include "prx/planning/planner_statistics.hpp"

using namespace boost::python;

void pyprx_planning_planner_statistics_py()
{

   	class_<prx::planner_statistics_t>("planner_statistics", no_init )//init<std::string, double>())   
   		.def("__init__", make_constructor(&init_as_ptr<prx::planner_statistics_t>, default_call_policies()) )
        .def("link_planner", &prx::planner_statistics_t::link_planner)
        .def("link_criterion", &prx::planner_statistics_t::link_criterion)
        .def("repeat_data_gathering", &prx::planner_statistics_t::repeat_data_gathering)
        .def("serialize", &prx::planner_statistics_t::serialize)
   	    ;
   	
}