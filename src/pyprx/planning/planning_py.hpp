#include <iostream>
#include <boost/python.hpp>
#include "pyprx/planning/world_model_py.hpp"
#include "pyprx/planning/condition_check_py.hpp"
#include "pyprx/planning/planners/planners_py.hpp"
#include "pyprx/planning/planner_statistics_py.hpp"
#include "pyprx/planning/planner_functions/planner_functions_py.hpp"

using namespace boost::python;
void pyprx_planning_py()
{

   	pyprx_planning_world_model_py();
   	pyprx_planning_condition_check_py();
   	pyprx_planning_planner_functions_py();
   	pyprx_planning_planner_statistics_py();
   	pyprx_planning_planners_py();
}