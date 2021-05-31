#include "prx/utilities/defs.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/loaders/planner_loader.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planner_statistics.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/simulation/system_factory.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
	try
	{
		std::string params_file;
		if (argc<=1)
		{
			prx_throw("The planner evaluation executable needs a parameter file!");
		}
		else
		{
			params_file = std::string(argv[1]);
		}

		param_loader params(params_file);

		simulation_step = params["simulation_step"].as<double>();

		//Planner parameters
		int planner_iterations = params["planner_iterations"].as<int>();
		int stats_iterations = params["statistics_iterations"].as<int>();
		int random_seed = params["random_seed"].as<int>();
		init_random(random_seed);

		//which plant we are planning for
		std::string plant_name = params["plant_name"].as<std::string>();
		std::string plant_type = params["plant_type"].as<std::string>();
		std::string obstacles_file = params["obstacles_file"].as<std::string>();
		auto obstacles = load_obstacles(obstacles_file);
		auto obstacle_list = obstacles.second;
		auto obstacle_names = obstacles.first;
		auto plant = system_factory_t::create_system(plant_type,plant_name);

		world_model_t<> world_model({plant},{obstacle_list});
		world_model.create_context("planning_context",{plant_name},{obstacle_names});
		auto context = world_model.get_context("planning_context");

		auto planner_wrapper = create_planner_wrapper(	"planning_context",
														context,
														param_loader(params["planner_parameters"]));

		bool get_visualization = params["visualize"].as<bool>();

		//start and goal states
		std::vector<double> start_vec = params["start_state"].as<std::vector<double>>();
		std::vector<double> goal_vec = params["goal_state"].as<std::vector<double>>();
		planner_wrapper->planner_query->start_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(planner_wrapper->planner_query->start_state,start_vec);
		planner_wrapper->planner_query->goal_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(planner_wrapper->planner_query->goal_state,goal_vec);
		planner_wrapper->planner_query->goal_region_radius = params["goal_region_radius"].as<double>();
		planner_wrapper->planner_query->get_visualization = get_visualization;

		condition_check_t checker("iterations",  planner_iterations/stats_iterations);
		std::ofstream fout;

		int stats_runs = params["planner_runs"].as<int>();
		for (int i = 0; i < stats_runs; i++)
		{				
			planner_wrapper->planner->link_and_setup_spec(planner_wrapper->planner_spec);
			planner_wrapper->planner->preprocess();
			planner_wrapper->planner->link_and_setup_query(planner_wrapper->planner_query);
			
			planner_statistics_t stats;
			stats.link_planner(planner_wrapper->planner);
			stats.link_criterion(&checker);
			stats.repeat_data_gathering(stats_iterations);

			std::string full_filename = lib_path+params["data_output_folder"].as<std::string>()+params["planner_parameters"]["planner_name"].as<std::string>()+"_"+std::to_string(i)+".txt";
			fout.open(full_filename);
			fout<<stats.serialize() << std::endl;
			fout.close();

			if(get_visualization)
			{
				planner_wrapper->planner->fulfill_query();
				std::string vis_body = params["visualization_body"].as<std::string>();
				three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

				for(auto& traj : planner_wrapper->planner_query->tree_visualization)
				{
					vis_group->add_vis_infos(info_geometry_t::LINE, traj, vis_body, context.first->get_state_space());
				}

				if (planner_wrapper->planner_query->solution_cost > 0)
				// if (planner_wrapper->planner_query->solution_plan.duration() > 0)
				{
					vis_group->add_vis_infos(info_geometry_t::LINE, planner_wrapper->planner_query->solution_traj, vis_body, context.first->get_state_space(),"0x00ffff");
					double timestamp=0;
					for(auto state : planner_wrapper->planner_query->solution_traj)
					{
						context.first->get_state_space()->copy_from_point(state);
						vis_group->snapshot_state(timestamp);
						timestamp+=simulation_step;
					}
				}
				else
				{
					double timestamp=0;
					context.first->get_state_space()->copy_from_point(planner_wrapper->planner_query->start_state);
					vis_group->snapshot_state(timestamp);
					timestamp+=simulation_step;
				}
				vis_group->output_html(params["output_html"].as<std::string>()+"_"+std::to_string(i)+".html");
				delete vis_group;
			}
			planner_wrapper->planner_query->clear_outputs();
			planner_wrapper->planner->reset();
		}
		delete planner_wrapper;
	}
	catch(const prx_assert_t& e)
	{
	}
}