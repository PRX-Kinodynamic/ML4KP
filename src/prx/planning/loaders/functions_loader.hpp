#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/planning/planner_functions/configurable_functions.hpp"

namespace prx
{
	// template< class SGM = system_group_manager_t, class CC = collision_checker_t> 
	planner_functions_t* create_planner_functions(	std::string planner_context, world_model_context context, param_loader pl)
	{
		std::string planner_function_name = pl["planner_function_name"].as<std::string>();
		if(planner_function_name=="default")
		{
			return new planner_functions_t(planner_context,context,pl);
		}
		else if(planner_function_name=="configurable")
		{
			return new configurable_functions_t(planner_context,context,pl);
		}
		else
		{
			prx_throw("Can't create a planner functions of type "<<planner_function_name);
		}
	}
}