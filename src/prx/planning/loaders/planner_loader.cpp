#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/planning/loaders/functions_loader.hpp"


namespace prx
{
	// template< class SGM, class CC> 
	planner_wrapper_t* create_planner_wrapper(	std::string planner_context,
												world_model_context context,
												// world_model_t<SGM, CC>* wm,
												param_loader pl)
	{
		planner_wrapper_t* wrapper = new planner_wrapper_t();
		// auto context = wm->get_context(planner_context);
		auto sg = context.first;
		auto cg = context.second;
		auto state_space = sg->get_state_space();
		auto control_space = sg->get_control_space();

		std::string planner_type = pl["planner_type"].as<std::string>();
		std::string planner_name = pl["planner_name"].as<std::string>();
		param_loader planner_functions_loader = param_loader(pl["planner_functions"]);

		if(planner_type=="rrt")
		{
			wrapper->planner = new rrt_t(planner_name);
			wrapper->planner_functions = create_planner_functions(planner_context,context,planner_functions_loader);
			auto spec = new rrt_specification_t(sg,cg);

			spec->cost_function = wrapper->planner_functions->cost_function;
			spec->distance_function = wrapper->planner_functions->distance_function;
			spec->sample_state = wrapper->planner_functions->sample_state;
			spec->sample_plan = wrapper->planner_functions->sample_plan;
			spec->valid_state = wrapper->planner_functions->valid_state;
			spec->valid_check = wrapper->planner_functions->valid_check;
			spec->propagate = wrapper->planner_functions->propagate;

			wrapper->planner_spec = spec;
			wrapper->planner_query = new rrt_query_t(state_space,control_space);
			return wrapper;
		}
		else if(planner_type=="sst")
		{
			wrapper->planner = new sst_t(planner_name);
			wrapper->planner_functions = create_planner_functions(planner_context,context,planner_functions_loader);
			auto spec = new sst_specification_t(sg,cg);

			spec->delta_near=pl["delta_near"].as<double>();
			spec->delta_drain=pl["delta_drain"].as<double>();

			spec->cost_function = wrapper->planner_functions->cost_function;
			spec->distance_function = wrapper->planner_functions->distance_function;
			spec->sample_state = wrapper->planner_functions->sample_state;
			spec->sample_plan = wrapper->planner_functions->sample_plan;
			spec->valid_state = wrapper->planner_functions->valid_state;
			spec->valid_check = wrapper->planner_functions->valid_check;
			spec->propagate = wrapper->planner_functions->propagate;

			wrapper->planner_spec = spec;
			wrapper->planner_query = new sst_query_t(state_space,control_space);
			return wrapper;
		}
		else if(planner_type=="dirt")
		{
			wrapper->planner = new dirt_t(planner_name);
			wrapper->planner_functions = create_planner_functions(planner_context,context,planner_functions_loader);
			auto spec = new dirt_specification_t(sg,cg);
			
			spec->blossom_number=pl["blossom_number"].as<int>();
			spec->use_pruning=pl["use_pruning"].as<bool>();

			spec->cost_function = wrapper->planner_functions->cost_function;
			spec->distance_function = wrapper->planner_functions->distance_function;
			spec->sample_state = wrapper->planner_functions->sample_state;
			spec->sample_plan = wrapper->planner_functions->sample_plan;
			spec->valid_state = wrapper->planner_functions->valid_state;
			spec->valid_check = wrapper->planner_functions->valid_check;
			spec->propagate = wrapper->planner_functions->propagate;
			spec->h = wrapper->planner_functions->h;
			spec->expand = wrapper->planner_functions->expand;

			wrapper->planner_spec = spec;
			wrapper->planner_query = new dirt_query_t(state_space,control_space);
			return wrapper;
		}
		else if(planner_type=="rastar")
		{
			wrapper->planner = new randomized_astar_t(planner_name);
			wrapper->planner_functions = create_planner_functions(planner_context,context,planner_functions_loader);
			auto spec = new rastar_specification_t(sg,cg);

			spec->blossom_number=pl["blossom_number"].as<int>();
			spec->delta_prune=pl["delta_prune"].as<double>();

			spec->cost_function = wrapper->planner_functions->cost_function;
			spec->distance_function = wrapper->planner_functions->distance_function;
			spec->sample_state = wrapper->planner_functions->sample_state;
			spec->sample_plan = wrapper->planner_functions->sample_plan;
			spec->valid_state = wrapper->planner_functions->valid_state;
			spec->valid_check = wrapper->planner_functions->valid_check;
			spec->propagate = wrapper->planner_functions->propagate;
			spec->h = wrapper->planner_functions->h;
			spec->expand = wrapper->planner_functions->expand;

			wrapper->planner_spec = spec;
			wrapper->planner_query = new rastar_query_t(state_space,control_space);
			return wrapper;
		}
		else
		{
			prx_throw("Can't create a planner of type "<<planner_type);
		}
	}
}