#include <iostream>
#include <boost/python.hpp>
#include "pyprx/simulation/playback/trajectory_py.hpp"
#include "pyprx/simulation/playback/plan_py.hpp"

void pyprx_simulation_playback_py()
{

	pyprx_simulation_playback_trajectory_py();
	pyprx_simulation_playback_plan_py();
}