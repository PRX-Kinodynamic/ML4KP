#include <iostream>
#include <boost/python.hpp>

#include "pyprx/pyprx_utils.hpp"
#include "pyprx/stdlib_py.hpp"
#include "pyprx/simulation/simulation_py.hpp"
#include "pyprx/utilities/utilities_py.hpp"
#include "pyprx/planning/planning_py.hpp"
#include "pyprx/visualization/visualization_py.hpp"



BOOST_PYTHON_MODULE(libpyDirtMP)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	pyprx_stdlib_py();
	pyprx_simulation_py();
	pyprx_utilities_py();
	pyprx_planning_py();
	pyprx_visualization_py();
   	// class_<prx::plant_t>("plant_t", init<std::string>())
   	    // .def("propagate", pure_virtual(&prx::system_t::propagate))
   	// ;
}