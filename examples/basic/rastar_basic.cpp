#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/plants.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/general/param_loader.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/planning/planners/randomized_astar.hpp"

#include <fstream>

using namespace prx;

int main(int argc, char* argv[])
{
    auto params = param_loader("examples/basic/RAstar.yaml", argc, argv);

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
    world_model.create_context("dirt_context",{plant_name},{obstacle_names});
    auto context = world_model.get_context("dirt_context");

    randomized_astar_t rastar(params["planner"].as<>());
    rastar_specification_t rastar_spec(context.first,context.second);

    // rastar_spec.valid_state = [](space_point_t& s)
    // {
        // Custom valid_state can be added here.  
    // };

    // rastar_spec.valid_check = [&rastar_spec](trajectory_t& traj)
    // {
        // Custom valid_check goes here...
        // Basically for x in traj, call valid_state
    // };

    // Two ways of accessing lengthy parameter paths
    int min_steps = params["plant"]["min_steps"].as<int>();
    int max_steps = params["/plant/max_steps"].as<int>();

    // rastar_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
    // {
        // Add custom sample plan here
    // };

        
    params["max_speed"].set(system_factory_t::get_system_max_velocity(plant_name, plant));
    const double max_vel = params["max_speed"].as<double>();
    rastar_spec.h = [&](const space_point_t& s, const space_point_t& s2)
    {
        // Custom h function: ( distance from s to s2 ) / (max velocity)
        return space_t::euclidean_2d(s, s2) / max_vel;
    };

    rastar_spec.min_control_steps  = min_steps;
    rastar_spec.max_control_steps  = max_steps;
    rastar_spec.blossom_number     = params["blossom"].as<int>();
    rastar_spec.delta_prune        = params["delta_prune"].as<double>();

    rastar_query_t rastar_query(context.first -> get_state_space(), context.first -> get_control_space());
    rastar_query.start_state = context.first->get_state_space() -> make_point();
    rastar_query.goal_state  = context.first->get_state_space() -> make_point();

    auto lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
    auto upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();
    context.first -> get_state_space() -> set_bounds(lower_bounds, upper_bounds);

    context.first -> get_state_space() -> copy_point_from_vector(rastar_query.start_state, params["/plant/start_state"].as<std::vector<double>>());
    context.first -> get_state_space() -> copy_point_from_vector(rastar_query.goal_state, params["/plant/goal_state"].as<std::vector<double>>());
    
    rastar_query.goal_region_radius = params["goal_region_radius"].as<double>();

    // Alternatively, change the goal_check function
    // rrt_query.goal_check = [&](space_point_t pt)
    // {
    //    // Default is:
        // return space_t::euclidean_2d(pt, rrt_query.goal_state) < goal_region_radius;
    // }

    rastar_query.get_visualization = params["visualize"].as<bool>();

    rastar.link_and_setup_spec(&rastar_spec);
    rastar.preprocess();
    rastar.link_and_setup_query(&rastar_query);

    condition_check_t checker(params["checker_type"].as<>(), params["checker_value"].as<int>()); //'

    rastar.resolve_query(&checker);
    rastar.fulfill_query(); 

    params.print();
        
    // TODO: Add function to visualization to replace tree_to_txt
    // tree_to_txt(dirt_query);
        
    three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
    
    std::string body_name = params["/plant/name"].as<>() + "/" + params["/plant/vis_body"].as<>();
    auto ss = context.first -> get_state_space();

    vis_group -> add_vis_infos(info_geometry_t::LINE, rastar_query.tree_visualization, body_name, ss);

    vis_group -> add_detailed_vis_infos(info_geometry_t::FULL_LINE, rastar_query.solution_traj, body_name, ss);

    vis_group -> add_animation(rastar_query.solution_traj, ss, rastar_query.start_state);

    vis_group -> output_html("output.html");

    delete vis_group;

    std::cout<<"End of program"<<std::endl;
}
