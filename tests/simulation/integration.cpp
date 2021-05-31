#define BOOST_AUTO_TEST_MAIN integrators
#include <string>
#include <boost/test/unit_test.hpp>


#include "prx/planning/world_model.hpp"

// Using acrobot due to its trajectories diverging fasts 
// when changing integration step
#include "prx/simulation/plants/two_link_acrobot.hpp"


BOOST_AUTO_TEST_CASE( integrators )
{
    auto acrobot_euler = prx::system_factory_t::create_system("Acrobot", "Acrobot_euler");
    auto acrobot_rk4 = prx::system_factory_t::create_system("Acrobot", "Acrobot_rk4");
    auto acrobot_dorpi = prx::system_factory_t::create_system("Acrobot", "Acrobot_dorpi");
	
     std::dynamic_pointer_cast<prx::plant_t>(acrobot_euler) -> set_integrator(prx::integrator_t::kEULER);
     std::dynamic_pointer_cast<prx::plant_t>(acrobot_rk4) -> set_integrator(prx::integrator_t::kRK4);
     std::dynamic_pointer_cast<prx::plant_t>(acrobot_dorpi) -> set_integrator(prx::integrator_t::kDOPRI5);

    std::vector<std::vector<double> > ctrls = {{6.94678,0.12,0.12},
                                               {6.70279,0.28,0.4},
                                               {6.22159,0.33,0.73},
                                               {-3.61730,0.36,1.09},
                                               {-5.02736,0.23,1.32},
                                               {-1.41110,0.2,1.52},
                                               {-6.98014,0.43,1.95},
                                               {-6.73070,0.2,2.15},
                                               {-3.74701,0.1,2.25},
                                               {2.47204,0.29,2.54},
                                               {3.82880,0.37,2.91},
                                               {-4.03203,0.01,2.92},
                                               {-2.47204,0.43,3.35}};

    prx::distance_function_t dist = [](prx::space_point_t a, prx::space_point_t b)
    {
        double res = 0;
        for (int i = 0; i < a -> get_dim(); ++i)
        {
            res += std::pow( a -> at(i) - b -> at(i) , 2);
        }
        return std::sqrt(res);
    };

    prx::world_model_t<> world_model({acrobot_euler},{});
    world_model.create_context("disk_context",{"Acrobot_euler"},{});
    auto context = world_model.get_context("disk_context");
    auto sg = context.first;
    auto state_space = context.first->get_state_space();
    auto control_space = context.first->get_control_space();

    prx::space_point_t curr_state;
    prx::plan_t plan_euler(control_space);
    prx::trajectory_t euler_traj(state_space);
    

    prx::world_model_t<> world_model_rk4({acrobot_rk4},{});
    world_model_rk4.create_context("context_rk4",{"Acrobot_rk4"},{});
    auto context_rk4 = world_model_rk4.get_context("context_rk4");
    auto sg_rk4 = context.first;
    auto state_space_rk4 = context.first->get_state_space();
    auto control_space_rk4 = context.first->get_control_space();

    prx::plan_t plan_rk4(control_space_rk4);
    prx::trajectory_t traj_rk4(state_space_rk4);

    prx::world_model_t<> world_model_dorpi({acrobot_dorpi},{});
    world_model_dorpi.create_context("context_dorpi",{"Acrobot_dorpi"},{});
    auto context_dorpi = world_model_dorpi.get_context("context_dorpi");
    auto sg_dorpi = context.first;
    auto state_space_dorpi = context_dorpi.first->get_state_space();
    auto control_space_dorpi = context_dorpi.first->get_control_space();

    prx::plan_t plan_dorpi(control_space_dorpi);
    prx::trajectory_t traj_dorpi(state_space_dorpi);

    for (auto ctrl : ctrls)
    {
        plan_euler.append_onto_back(ctrl[1]);
        plan_euler.back().control -> at(0) = ctrl[0];

        plan_rk4.append_onto_back(ctrl[1]);
        plan_rk4.back().control -> at(0) = ctrl[0];

        plan_dorpi.append_onto_back(ctrl[1]);
        plan_dorpi.back().control -> at(0) = ctrl[0];
    }

    curr_state = state_space -> make_point();
    curr_state -> at(0) = 0.0;
    curr_state -> at(1) = 0.0;
    curr_state -> at(2) = 0.0;
    curr_state -> at(3) = 0.0;

    prx::simulation_step=0.000001;
    sg -> propagate(curr_state, plan_euler, euler_traj);
    prx::simulation_step=0.001;
    sg_rk4 -> propagate(curr_state, plan_rk4, traj_rk4);

    prx::simulation_step=0.001;
    sg_dorpi -> propagate(curr_state, plan_dorpi, traj_dorpi);

    for (double i = 0; i <= 1; i+=0.01)
    {
        auto pt_euler = euler_traj.at(i);
        auto pt_rk4 = traj_rk4.at(i);
        auto pt_dorpi = traj_dorpi.at(i);

        BOOST_CHECK(dist(pt_rk4, pt_euler) < 0.1);
        BOOST_CHECK(dist(pt_rk4, pt_dorpi) < 0.01);
        // std::cout << "euler: " << pt_euler << "\trk4: " << pt_rk4 << std::endl;
        // std::cout << "dorpi: " << pt_dorpi << "\trk4: " << pt_rk4 << std::endl;
    }


}