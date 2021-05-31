#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/dirt.hpp"
#include "prx/simulation/system_factory.hpp"
#include "prx/planning/planners/planner.hpp"
#include "prx/visualization/three_js_group.hpp"
#include "prx/utilities/heuristics/medial_axis.hpp"
// #include "prx/utilities/data_structures/heuristic_map.hpp"
#include "prx/simulation/system_factory.hpp"

#include "prx/simulation/loaders/obstacle_loader.hpp"
// #include "corl/common.cpp"

using namespace prx;
// const std::string map_fname = input_path + "/maps/" + variables["map_name"];
const std::string sknw_file = lib_path + "/out/ma_sknw_" + "simple_obstacle" + ".txt";
const std::string nodes_file = lib_path + "/out/ma_sknw_nodes_" + "simple_obstacle" + ".txt";
const std::string edges_file = lib_path + "/out/ma_sknw_edges_" + "simple_obstacle" + ".txt";
const std::string file_far = lib_path + "/out/ma_map_" + "simple_obstacle" + "_vf_far.txt";
const std::string file_close = lib_path + "/out/ma_map_" + "simple_obstacle" + "_vf_close.txt";
const std::string integrated_file = lib_path + "/out/ma_map_" + "simple_obstacle" + "_vf_integrated.txt";
const std::string blossom_file = lib_path + "/out/ma_map_" + "simple_obstacle" + "_vf_blossom.txt";
const std::string cost_to_go_file = lib_path + "/out/ma_map_" + "simple_obstacle" + "_vf_cost_to_go.txt";


// This resolution is for rrt_star_obstacles
// Lower resolution is faster but could lead to problems
// with narrow passages.
const double map_resolution = 0.05;

int main(int argc, char* argv[])
{
    std::vector<double> lower;
    std::vector<double> upper;

    try
    {
        auto obstacles = load_obstacles("obstacles/rrt_star_obstacles.yaml");

        simulation_step = 0.1;

        std::vector<std::shared_ptr<movable_object_t>> obstacle_list = obstacles.second;
        std::vector<std::string> obstacle_names = obstacles.first;

        init_random(231192);
        std::string plant_name = "treaded_vehicle_fo";
        system_ptr_t plant = system_factory_t::create_system(plant_name, plant_name);

        world_model_t<> world_model({plant},{obstacle_list});

        world_model.create_context("disk_context",{plant_name},{obstacle_names});

        lower = {-10.0,-10.0,-M_PI};
        upper = { 10.0, 10.0,M_PI}; 
        plant->set_state_space_bounds(lower,upper);

        // const double max_vel = 0.7;

        auto context = world_model.get_context("disk_context");

        dirt_t dirt("dirt");
        dirt_specification_t dirt_spec(context.first,context.second);

        int min_steps = 10;
        int max_steps = 100;
        // Assuming goal is in (9, 9)
        std::vector<double> goal_vec = {(9.0 - lower[0])/map_resolution, (9.0 - lower[1])/map_resolution};

        dirt_spec.min_control_steps = min_steps;
        dirt_spec.max_control_steps = max_steps;
        dirt_spec.blossom_number = 5;
        dirt_spec.use_pruning = false;


        medial_axis_t* medial_axis = new medial_axis_t();

        // Function needed to discretize the environment
        mapping_f func = [&](int i, int j, space_point_t p)
        {
            p -> at(0) = ( i * map_resolution + lower[0] ) ;
            p -> at(1) = ( j * map_resolution + lower[1] ) ;
        };

        // Init the MA
        medial_axis -> init();

        // Set the environment. 
        medial_axis -> set_map((upper[0] - lower[0])/map_resolution, (upper[1] - lower[1])/map_resolution,
            dirt_spec.valid_state, func, context.first->get_state_space());

        // The following 5 functions are needed in order:
        medial_axis -> compute_close_vector_field(); 
        medial_axis -> set_goal(goal_vec);
        medial_axis -> find_medial_axis();
        medial_axis -> prepare_graph();
        medial_axis -> compute_far_vector_field();

        // After this, the MA can be called directly or save to files:
        // Save to files
        medial_axis -> close_vf_to_file(file_close, false);
        medial_axis -> edges_to_file(edges_file);
        medial_axis -> nodes_to_file(nodes_file);
        medial_axis -> far_vf_to_file(file_far, false);
        medial_axis -> integrated_vf_to_file(integrated_file);
        medial_axis -> blossom_to_file(blossom_file);
        medial_axis -> cost_to_go_to_file(cost_to_go_file);
        
        double multiplier = simulation_step >= 1 ? simulation_step : 1./simulation_step;

        double max_vel = system_factory_t::get_system_max_velocity(plant_name, plant);
        dirt_spec.h = [&](space_point_t pose, space_point_t goal)
        {
            double w = (pose -> at(0) - lower[0])/map_resolution;
            double h = (pose -> at(1) - lower[1])/map_resolution;
            double c = medial_axis -> cost_to_go(w,h);
            return c / max_vel;
        };

        // The alternative is to call a single coordinate of the MA:
        // medial_axis -> integrated_vector_at(w,h,max_vel);
        // See the lines below
        dirt_spec.sample_plan = [&](plan_t& plan, space_point_t pose)
        {
            // (h,w) are the coordinates of pose in the MA matrix
            double h = (pose -> at(1) - lower[1])/map_resolution;
            double w = (pose -> at(0) - lower[0])/map_resolution;

            // Get a random time to propagate
            double delta_t = uniform_int_random(min_steps,max_steps)/multiplier;

            // Get the integrated vector
            std::complex<double> vp = medial_axis -> integrated_vector_at(w,h,max_vel*delta_t);
            // Look-ahead: see where the next vector is pointing to 
            std::complex<double> look_ahead = std::complex<double>(w,h) + vp;

            // This is redundant. For the city maps, a swaping is necessary
            std::complex<double> u = medial_axis -> integrated_vector_at(look_ahead.real(), look_ahead.imag());
            
            // Get the angle difference 
            double diff_theta = norm_angle_pi(-std::arg(u) + pose -> at(2));

            // The distance between wheels for the FO treaded vehicle
            double wi = 0.6;
            // vr <- cos(d_Angle) ==> no diff (0) means full ahead
            double vr = std::cos(diff_theta);
            double vl = ( diff_theta / delta_t ) * wi + vr;
            double max_v = std::max( std::abs(vr), std::abs(vl));

            // Get the velocities within bounds
            vr = 0.7 * ( vr / max_v );
            vl = 0.7 * ( vl / max_v );
            
            // Add the computed vels to the plan
            plan.append_onto_back(delta_t);
            plan.back().control -> at(0) = vl;
            plan.back().control -> at(1) = vr;
            
            // Extra precaution...
            dirt_spec.control_space -> enforce_bounds(plan.back().control);
        };


        dirt_query_t dirt_query(context.first->get_state_space(), context.first->get_control_space());
        dirt_query.start_state = context.first->get_state_space()->make_point();
        dirt_query.goal_state  = context.first->get_state_space()->make_point();

        for (int i = 0; i < context.first -> get_state_space() -> get_dimension(); ++i)
        {
            dirt_query.start_state -> at(i) = 0.0;
            dirt_query.goal_state -> at(i) = 0.0;
        }

        dirt_query.start_state -> at(0) = -9.5;
        dirt_query.start_state -> at(1) = -9.5;

        dirt_query.goal_state -> at(0) = 9;//10.0;
        dirt_query.goal_state -> at(1) = 9;//17.50;

        dirt_query.goal_region_radius = 0.5;
        dirt_query.get_visualization = true;

        dirt.link_and_setup_spec(&dirt_spec);
        dirt.preprocess();
        dirt.link_and_setup_query(&dirt_query);

        condition_check_t checker("iterations", 1000); //'

        dirt.resolve_query(&checker);
        dirt.fulfill_query();

        // print_variables();

        // tree_to_txt(dirt_query);

        three_js_group_t* vis_group = new three_js_group_t({plant},{obstacle_list});
        
        for(auto& traj : dirt_query.tree_visualization)
        {
            if(traj != dirt_query.solution_traj)
            vis_group->add_vis_infos(info_geometry_t::FULL_LINE, traj, plant_name+"/body", context.first->get_state_space());
        }
        if(dirt_query.solution_traj.size()!=0)
        {
            vis_group->add_vis_infos(info_geometry_t::FULL_LINE, dirt_query.solution_traj,
                plant_name+"/body", context.first->get_state_space(), "0xFF0000");
        }

        double timestamp=0;
        for(auto state : dirt_query.solution_traj)
        {
            context.first->get_state_space() -> copy_from_point(state);
            vis_group->snapshot_state(timestamp);
            timestamp+=simulation_step;
        }
        if(dirt_query.solution_traj.size()==0)
        {
            context.first->get_state_space() -> copy_from_point(dirt_query.start_state);
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
}
