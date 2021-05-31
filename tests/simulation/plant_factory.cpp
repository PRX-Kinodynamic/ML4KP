#define BOOST_AUTO_TEST_MAIN system_factory_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/simulation/plants/plants.hpp"

BOOST_AUTO_TEST_CASE( system_factory_test )
{
	std::vector<std::string> system_names = 
		{"2D_Point", "rally_car", "treaded_vehicle", "3D_Point", "Acrobot", "fixed_wing", "omni_mantis", "koules", "racecar_mini", "FO_treaded_vehicle", "Ackermann_FO"};

    std::vector<std::pair<std::string, std::string>> fn_names = 
        {{"2D_Point", "2D_Point"}, {"rally_car", "rally_car"}, {"treaded_vehicle", "treaded_vehicle"}, {"FO_treaded_vehicle", "FO_treaded_vehicle"}};


    for (auto name : prx::system_factory_t::available_systems())
    {
        printf("\tChecking available system: %s...", name.c_str() );
        BOOST_CHECK(std::find(system_names.begin(), system_names.end(), name) != system_names.end());
        printf("\t[ OK ]\n" );
    }

    printf("Checking systems in Factory...\n");
	for (auto name : system_names)
    {
      	printf("\tChecking systems: %s...", name.c_str() );
    	std::string path = name + "_path";  
      	auto sys_ptr = prx::system_factory_t::create_system(name, path);
      	BOOST_CHECK( sys_ptr != nullptr ); 
      	BOOST_CHECK( sys_ptr -> get_pathname() == path); 
      	printf("\t[ OK ]\n" );
    }

    printf("\nChecking functions in Factory...\n");
    for (auto p : fn_names)
    {
        printf("\tChecking function: %s...", p.first.c_str() );
        BOOST_CHECK(prx::system_factory_t::get_system_max_velocity(p.first, prx::system_factory_t::create_system(p.second, p.second)) < std::numeric_limits<double>::infinity());
        printf("\t[ OK ]\n" );
    }

}