#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

#include "prx/planning/planners/rrt.hpp"
#include "prx/planning/planners/sst.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/planning/planners/randomized_astar.hpp"

namespace prx
{

	struct planner_wrapper_t
	{
		planner_t* planner;
		planner_specification_t* planner_spec;
		planner_query_t* planner_query;
		planner_functions_t* planner_functions;

		~planner_wrapper_t()
		{
			delete planner_query;
			delete planner_functions;
			delete planner_spec;
			delete planner;
		}
	};

	planner_wrapper_t* create_planner_wrapper(	std::string planner_context, world_model_context context, param_loader pl);
}