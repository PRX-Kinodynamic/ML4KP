#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>

#include "prx/utilities/spaces/space.hpp"
// #include "prx/simulation/plants/two_link_acrobot.hpp"
// #include "prx/simulation/plant.hpp"
 
using namespace boost::python;

int get_dim_wrapper(prx::space_point_t p)
{
    return p -> get_dim();
}
double space_point_get_item(prx::space_point_t& pt, int i)
{
    INDEX_CHECK(i, pt -> get_dim(), "space_point" )

    return pt -> at(i);
}
void space_point_set_item(prx::space_point_t& pt, int i, double val)
{
    INDEX_CHECK(i, pt -> get_dim(), "space_point" )

    pt -> at(i) = val;
}



// object init_distance_function()
prx::distance_function_t init_distance_function()
{
    prx::distance_function_t default_df = [](const prx::space_point_t& s1, const prx::space_point_t& s2)
    {
        return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
                    (s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)));
    };
    // return boost::python::make_function(default_df);
    return default_df;
}

struct distance_function_wrapper : prx::distance_function_t, wrapper<prx::distance_function_t>
{

};

// void set_distance_function(prx::distance_function_t& self, boost::python::object obj)
// {
//     self = 
// }
// spam& self, boost::python::object object

prx::distance_function_t get_df(PyObject* x)
{

    std::function<double (const prx::space_point_t&, const prx::space_point_t&)> new_df = [x](const prx::space_point_t& s1, const prx::space_point_t& s2)
    {
        return call<double>(x, s1, s2);
    };
    return new_df;
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(space_t_print_point_overloads, prx::space_t::print_point, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(space_t_print_memory_overloads, prx::space_t::print_memory, 0, 1)


void  (prx::space_t::*enforce_bounds0)() const = &prx::space_t::enforce_bounds;
void  (prx::space_t::*enforce_bounds1)(const prx::space_point_t&) const = &prx::space_t::enforce_bounds;

void  (prx::space_t::*integrate_0)(const prx::space_point_t&, const prx::space_t*, double) = &prx::space_t::integrate;
void  (prx::space_t::*integrate_1)(const prx::space_t*, double) = &prx::space_t::integrate;

void pyprx_utilities_spaces_space()
{

   // typedef std::shared_ptr<space_snapshot_t> space_point_t;
    class_<prx::space_point_t>("space_point", no_init)
        .def("__len__", &get_dim_wrapper)
        .def("__getitem__", &space_point_get_item)
        .def("__setitem__", &space_point_set_item)
        // .def("assign", &list_assign<double>)
        .def("__str__", &prx_to_str<prx::space_point_t>) 
        .def("__repr__", &prx_print<prx::space_point_t>) 
        // .def(str(self))
        ;
    enum_<prx::space_t::topology_t>("topology")
        .value("EUCLIDEAN", prx::space_t::topology_t::EUCLIDEAN)
        .value("ROTATIONAL", prx::space_t::topology_t::ROTATIONAL)
        .value("DISCRETE", prx::space_t::topology_t::DISCRETE)
        .export_values()
        ;    

    class_<prx::distance_function_t>("distance_function")
        .def("__call__", &prx::distance_function_t::operator() )
        .def("default", make_function(&init_distance_function, default_call_policies())).staticmethod("default")
        .def("set_df", &get_df).staticmethod("set_df")
        .def("__setattr__", &get_df)
        ;


   	class_<prx::space_t, std::shared_ptr<prx::space_t>>("space_t", init<std::string, std::vector<double*>, std::string>())
        .def("__init__", make_constructor(&init_as_ptr<prx::space_t,std::string,std::vector<double*>>, default_call_policies(), (args("topology"), args("addresses")) ))
        .def("__init__", make_constructor(&init_as_ptr<prx::space_t,const std::vector<const prx::space_t*>>, default_call_policies(), (args("spaces")) ))
        .def("set_bounds", &prx::space_t::set_bounds)
        .def("make_point", &prx::space_t::make_point)
        .def("clone_point", &prx::space_t::clone_point)
        .def("enforce_bounds", enforce_bounds0)
   	    .def("enforce_bounds", enforce_bounds1)
        .def("at", &prx::space_t::at, return_value_policy<copy_non_const_reference>())
   	    .def("__getitem__", &prx::space_t::at, return_value_policy<copy_non_const_reference>())
   	    .def("get_dimension", &prx::space_t::get_dimension)
        .def("copy_to_point", &prx::space_t::copy_to_point)
        .def("copy_from_point", &prx::space_t::copy_from_point)
        // .def("copy_from_vector", &prx::space_t::copy_from_vector)
        .def("copy_point", &prx::space_t::copy_point)
        .def("copy_point_from_vector", &prx::space_t::copy_point_from_vector)
        .def("copy_vector_from_point", &prx::space_t::copy_vector_from_point)
        .def("is_point_in_space", &prx::space_t::is_point_in_space)
        .def("split_point", &prx::space_t::split_point)
   	    .def("print_memory", &prx::space_t::print_memory, space_t_print_memory_overloads())
        .def("print_point", &prx::space_t::print_point, space_t_print_point_overloads())
        .def("equal_points", &prx::space_t::equal_points)
        .def("get_dimension", &prx::space_t::get_dimension)
        .def("satisfies_bounds", &prx::space_t::satisfies_bounds)
        .def("sample", &prx::space_t::sample)
        .def("get_space_name", &prx::space_t::get_space_name)
        // .def("get_lower_bound", &prx::space_t::get_lower_bound)
        // .def("get_upper_bound", &prx::space_t::get_upper_bound)
        .def("get_bounds", &prx::space_t::get_bounds)
        .def("integrate", integrate_0)
        .def("integrate", integrate_1)
        .def("interpolate", &prx::space_t::interpolate)
        // .def("", &prx::space_t::)
        // .def("", &prx::space_t::)
        // .def("", &prx::space_t::)
        // .def("", &prx::space_t::)
   	    ;
}
