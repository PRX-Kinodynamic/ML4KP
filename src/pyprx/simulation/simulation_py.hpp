#include <iostream>
#include <boost/python.hpp>
#include "pyprx/simulation/plant_py.hpp"
#include "pyprx/simulation/plants/plants_py.hpp"
#include "pyprx/simulation/system_group_py.hpp"
#include "pyprx/simulation/system_py.hpp"
#include "pyprx/simulation/collision_checking/collision_checking_py.hpp"
#include "pyprx/simulation/playback/playback_py.hpp"

void pyprx_simulation_py()
{
	pyprx_simulation_system_py();
	pyprx_simulation_plant();
	pyprx_simulation_system_group_py();
	pyprx_simulation_collision_checking_py();

	pyprx_simulation_plants();
	pyprx_simulation_playback_py();
}