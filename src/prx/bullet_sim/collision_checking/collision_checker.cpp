#include "prx/bullet_sim/collision_checking/collision_checker.hpp"


namespace prx
{

	void bullet_collision_checker_t::add_collision_group(const std::string& group_name, 
		const std::vector<system_ptr_t>& in_plants, const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles)
	{
		collision_groups[group_name] = std::make_shared<bullet_collision_group_t>(in_plants,in_obstacles);
	}

}
