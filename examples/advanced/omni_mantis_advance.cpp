#include <unistd.h>
#include "prx/utilities/defs.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"
#include "prx/simulation/plants/omni_mantis.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/randomized_astar.hpp"
#include "prx/visualization/three_js_group.hpp"

using namespace prx;

int main(int argc, char* argv[])
{
	try
	{
		std::string params_file;
		if(argc<=1)
		{
			params_file = "omni_params.yaml";
		}
		else
		{
			params_file = std::string(argv[1]);
		}
		param_loader params(params_file);

		std::vector<double> start_vec = params["start_state"].as<std::vector<double>>();
		std::vector<double> goal_vec = params["goal_state"].as<std::vector<double>>();
		int iterations = params["planner_iterations"].as<int>();

		init_random(12345);
		simulation_step=0.01;
		system_ptr_t plant = create_system<omni_mantis_t>("mantis");

		transform_t obstacle_pose;
		obstacle_pose.setIdentity();
		obstacle_pose.translation() = (vector_t(10,10,0));
		auto box_obstacle = create_obstacle(new box_t("box",2,3,4,obstacle_pose));

		world_model_t<> world_model({plant},{box_obstacle});

		world_model.create_context("disk_context",{"mantis"},{"box"});

		auto context = world_model.get_context("disk_context");

		randomized_astar_t rastar("rastar");
		rastar_specification_t rastar_spec(context.first,context.second);

		rastar_spec.delta_prune = .3;

		rastar_spec.cost_function = [](const trajectory_t& traj, const plan_t& plan)
		{
			return plan.duration();
		};
		rastar_spec.distance_function = [](const space_point_t& s1, const space_point_t& s2)
		{ 
			return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
						(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)))/1.677;//dividing by 1.0 since that is the max vel of the vehicle.
		};

		rastar_spec.h = [&rastar_spec](const space_point_t& s, const space_point_t& s2)
		{
			return default_heuristic_function(s, s2, rastar_spec.distance_function);
		};

		rastar_query_t rastar_query(context.first->get_state_space(),context.first->get_control_space());
		rastar_query.start_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(rastar_query.start_state,start_vec);
		rastar_query.goal_state = context.first->get_state_space()->make_point();
		context.first->get_state_space()->copy_point_from_vector(rastar_query.goal_state,goal_vec);
		rastar_query.goal_region_radius = 1;
		rastar_query.get_visualization = true;
		bool get_visualization;
		rastar.link_and_setup_spec(&rastar_spec);
		rastar.preprocess();
		rastar.link_and_setup_query(&rastar_query);

		condition_check_t checker("iterations",  iterations);

		rastar.resolve_query(&checker);
		rastar.fulfill_query();


		std::cout<<"Solution Length: "<<rastar_query.solution_plan.duration()<<std::endl;

		three_js_group_t* vis_group = new three_js_group_t({plant},{box_obstacle});

		for(auto& traj : rastar_query.tree_visualization)
		{
			vis_group->add_vis_infos(info_geometry_t::LINE, traj, "mantis/chassis", context.first->get_state_space());
		}

		double timestamp=0;
		for(auto state : rastar_query.solution_traj)
		{
			context.first->get_state_space()->copy_from_point(state);
			vis_group->snapshot_state(timestamp);
			timestamp+=simulation_step;
		}
		if(rastar_query.solution_traj.size()==0)
		{
			context.first->get_state_space()->copy_from_point(rastar_query.start_state);
			vis_group->snapshot_state(timestamp);
			timestamp+=simulation_step;
		}

		vis_group->output_html("output.html");

		delete vis_group;


	}
	catch(const prx_assert_t& e)
	{
		std::cout<<e.get_message()<<std::endl;
	}
	std::cout<<"End of program"<<std::endl;
	_exit(1);
}