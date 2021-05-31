#define BOOST_AUTO_TEST_MAIN world_model_test
#include <string>
// TODO: Change to <boost/test/unit_test.hpp> 
#include <boost/test/unit_test.hpp>

#include "prx/planning/world_model.hpp"
#include "prx/simulation/plants/rally_car.hpp"
#include "prx/simulation/plants/treaded_vehicle.hpp"
#include "prx/simulation/plants/two_dimensional_point.hpp"
#include "prx/simulation/plants/three_dimensional_point.hpp"
#include "prx/simulation/plants/two_link_acrobot.hpp"
#include "prx/simulation/plants/fixed_wing.hpp"
#include "prx/simulation/plants/koules.hpp"
#include "prx/simulation/plants/omni_mantis.hpp"
#include "prx/simulation/plants/racecar_mini.hpp"
#include "prx/simulation/plants/treaded_vehicle_first_order.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"

BOOST_AUTO_TEST_CASE( world_model_test )
{
	std::vector<std::string> system_names = 
		{"2D_Point", "rally_car", "treaded_vehicle"};

    prx::transform_t obstacle_pose;
        obstacle_pose.setIdentity();
    auto box_obstacle = prx::create_obstacle(new prx::box_t("box",2,3,4,obstacle_pose));

    // std::vector<std::shared_ptr<prx::movable_object_t>> obstacle_list = obstacles.second;
    // std::vector<std::string> obstacle_names = obstacles.first;
    
    std::vector<prx::system_ptr_t> plants;

	for (auto name : system_names)
    {
      	auto sys_ptr = prx::system_factory_t::create_system(name, name);
        plants.push_back(sys_ptr);
    }
    // plants.push_back(prx::system_factory_t::create_system(system_names[0], system_names[0] + "_path"));
    // auto plant = prx::system_factory_t::create_system(system_names[0], system_names[0]);

    printf("Creating world model\n");
    prx::world_model_t<> world_model({plants},{box_obstacle});

    printf("Creating context\n");
    world_model.create_context("test_context",{system_names},{"box"});

    printf("Getting context\n");
    auto context = world_model.get_context("test_context");

    BOOST_CHECK(context.first != nullptr);
    BOOST_CHECK(context.second != nullptr);
    BOOST_CHECK(world_model.get_all_context_names()[0] == "test_context");


}