#include <iostream>
#include <boost/python.hpp>
#include "prx/simulation/system.hpp"
#include "prx/simulation/plant.hpp"
 
using namespace boost::python;



struct plant_wrap : prx::plant_t, wrapper<prx::plant_t>
{
    plant_wrap(const std::string& path) : prx::plant_t(path){};

    void update_configuration()
    {
        this->get_override("update_configuration")();
    }

public:
    void compute_derivative() 
    {
        this->get_override("compute_derivative")();
        // this -> compute_derivative();
    }

};

PRX_SETTER(plant_t, derivative_space)
PRX_GETTER(plant_t, derivative_space)

PRX_SETTER(plant_t, derivative_memory)
PRX_GETTER(plant_t, derivative_memory)

void pyprx_simulation_plant()
{
//    	class_<prx::plant_t>("plant_t", init<std::string>())
//    	    .def("propagate", &prx::plant_t::propagate)
//    	;
    // class_<plant_wrap, bases<prx::system_t, prx::movable_object_t>, boost::noncopyable>("plant_t", init<std::string>())
   	class_<plant_wrap, bases<prx::system_t>, boost::noncopyable>("plant_t", no_init)
      // .def("__init__", make_constructor(&init_as_ptr<prx::plant_t, std::string>, default_call_policies()))
      .def("add_system", &prx::plant_t::add_system)
      .def("propagate", &prx::plant_t::propagate)
      .def("compute_stopping_maneuver", &prx::plant_t::compute_stopping_maneuver)
      .def("compute_control", &prx::plant_t::compute_control)
      .def("update_configuration", pure_virtual(&prx::plant_t::update_configuration))
      // TODO: Add "std::pair<unsigned,unsigned>" class
      // .def("get_collision_list", &prx::plant_t::get_collision_list)
      .def("set_integrator", &prx::plant_t::set_integrator)
      .def("set_state_space_bounds", &prx::plant_t::set_state_space_bounds)
      .def("get_derivative_space", &get_plant_t_derivative_space<prx::space_t*>, return_internal_reference<>())
      .def("set_derivative_space", &set_plant_t_derivative_space<prx::space_t*>, return_internal_reference<>())
      // TODO: Add "std::vector<double*>" class
      // .def("derivative_memory", &get_plant_t_derivative_memory<std::vector<double*>>, &set_plant_t_derivative_memory<std::vector<double*>>)
   		;
        
    iterable_converter()
        .from_python<std::vector<prx::system_ptr_t> >()
   	    ;
	// pyprx_simulation_plants_acrobot();
   	
}
