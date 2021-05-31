#include <iostream>
#include <boost/python.hpp>
#include "prx/planning/planner_functions/planner_functions.hpp"

using namespace boost::python;

prx::heuristic_function_t init_heuristic_function()
{
    prx::heuristic_function_t default_h = [](const prx::space_point_t& s1, const prx::space_point_t& s2)
    {
        return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
                    (s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)));
    };
    // return boost::python::make_function(default_df);
    return default_h;
}

void pyprx_planning_planner_functions_py()
{

   	// class_<prx::heuristic_function_t>("heuristic_function")
    //     .def("__call__", &prx::heuristic_function_t::operator() )
    //     .def("default", make_function(&init_heuristic_function, default_call_policies())).staticmethod("default")
    //     .def("set_f", &create_function<prx::heuristic_function_t, double, const prx::space_point_t&, const prx::space_point_t&>).staticmethod("set_f")
    //     ;
    def("default_valid_state", &prx::default_valid_state);
    class_<prx::valid_state_t>("valid_state")
        .def("__call__", &prx::valid_state_t::operator())
        .def("set_f", &create_function<prx::valid_state_t, bool, prx::space_point_t>).staticmethod("set_f")
        ;

    // typedef std::function<bool (trajectory_t&)> valid_trajectory_t;
    def("default_valid_trajectory", &prx::default_valid_trajectory);
    class_<prx::valid_trajectory_t>("valid_trajectory")
        .def("__call__", &prx::valid_trajectory_t::operator())
        .def("set_f", &create_function<prx::valid_trajectory_t, bool, prx::trajectory_t>).staticmethod("set_f")
        ;
}