#include <iostream>
#include <boost/python.hpp>
#include "pyprx/planning/planners/planner_py.hpp"
// #include "pyprx/planning/planners/hyb_aorrt2_stride_py.hpp"
#include "pyprx/planning/planners/rrt_py.hpp"
#include "pyprx/planning/planners/sst_py.hpp"
#include "pyprx/planning/planners/dirt_py.hpp"

using namespace boost::python;
void pyprx_planning_planners_py()
{
	pyprx_planning_planners_planner_py();
	pyprx_planning_planners_rrt_py();
	pyprx_planning_planners_sst_py();
	pyprx_planning_planners_dirt_py();
   	// pyprx_planning_planners_hyb_aorrt2_stride_py();
}