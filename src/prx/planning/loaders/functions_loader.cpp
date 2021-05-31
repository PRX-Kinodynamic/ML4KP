#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/loaders/functions_loader.hpp"


namespace prx
{
	// planner_functions_t* create_planner_functions(	std::string planner_context,
	// 												world_model_t<>* wm,
	// 												param_loader pl)
	// {
	// 	std::string planner_function_name = pl["planner_function_name"].as<std::string>();
	// 	if(planner_function_name=="default")
	// 	{
	// 		return new planner_functions_t(planner_context,wm,pl);
	// 	}
	// 	else if(planner_function_name=="configurable")
	// 	{
	// 		return new configurable_functions_t(planner_context,wm,pl);
	// 	}
	// 	else
	// 	{
	// 		prx_throw("Can't create a planner functions of type "<<planner_function_name);
	// 	}
	// }
}