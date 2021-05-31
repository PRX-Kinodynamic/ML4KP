#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/basic/rrt.yaml", argc, argv);

    simulation_step = params["simulation_step"].as<double>();
    init_random(params["random_seed"].as<int>());

    auto obstacles = load_obstacles(params["environment"].as<>());
    std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
    std::vector<std::string> obstacle_names = obstacles.first;
        
    std::string plant_name = params["/plant/name"].as<>();
    std::string plant_path = params["/plant/path"].as<>();
    auto plant = prx::system_factory_t::create_system(plant_name, plant_path);
    prx_assert(plant != nullptr, "Plant is nullptr!");

    world_model_t<> world_model({plant},{obstacle_list});
    world_model.create_context("rrt_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("rrt_context");

    rrt_t rrt("rrt");
    rrt_specification_t rrt_spec(context.first,context.second);

    // rrt_spec.valid_state = [](space_point_t& s)
    // {
        // Custom valid_state can be added here.  
    // };

    // rrt_spec.valid_check = [&rrt_spec](trajectory_t& traj)
    // {
        // Custom valid_check goes here...
        // Basically for x in traj, call valid_state
    // };

    // Two ways of accessing lengthy parameter paths
    int min_steps = params["plant"]["min_steps"].as<int>();
    int max_steps = params["/plant/max_steps"].as<int>();

    // rrt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
    // {
        // Add custom sample plan here
    // };
    
    rrt_spec.min_control_steps = min_steps;
    rrt_spec.max_control_steps = max_steps;

    rrt_query_t rrt_query(context.first -> get_state_space(), context.first -> get_control_space());
    rrt_query.start_state = context.first->get_state_space() -> make_point();
    rrt_query.goal_state  = context.first->get_state_space() -> make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

    context.first -> get_state_space() -> copy_point_from_vector(rrt_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(rrt_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());

    rrt_query.goal_region_radius = params["goal_region_radius"].as<double>();
    
    // Alternatively, change the goal_check function
    // rrt_query.goal_check = [&](space_point_t pt)
    // {
    //    // Default is:
        // return space_t::euclidean_2d(pt, rrt_query.goal_state) < goal_region_radius;
    // }

    rrt_query.get_visualization = params["visualize"].as<bool>();

    rrt.link_and_setup_spec(&rrt_spec);
    rrt.preprocess();
    rrt.link_and_setup_query(&rrt_query);

    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>()); //'

    rrt.resolve_query(&checker);
    rrt.fulfill_query(); 

    params.print();
        
        // TODO: Add function to visualization to replace tree_to_txt
        // tree_to_txt(dirt_query);
        
    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});

    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    auto ss = context.first -> get_state_space();

    vis_group -> add_vis_infos(info_geometry_t::LINE, rrt_query.tree_visualization, 
        body_name, ss);

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, rrt_query.solution_traj, 
        body_name, ss);

    vis_group -> add_animation(rrt_query.solution_traj, ss, rrt_query.start_state);

    vis_group -> output_html("output.html");
    
    delete vis_group;

    std::cout<<"End of program"<<std::endl;
}
