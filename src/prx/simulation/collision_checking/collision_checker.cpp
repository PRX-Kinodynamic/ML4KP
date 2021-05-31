#include "prx/simulation/collision_checking/collision_checker.hpp"


namespace prx
{
	
	collision_checker_t::collision_checker_t()
	{
	}
	collision_checker_t::~collision_checker_t()
	{
	}

	void collision_checker_t::add_collision_group(const std::string& group_name, 
													const std::vector<system_ptr_t>& in_plants,
													const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles)
	{
		collision_groups[group_name] = std::make_shared<collision_group_t>(in_plants,in_obstacles);
	}


	std::shared_ptr<collision_group_t> collision_checker_t::get_collision_group(const std::string& group_name)
	{
		prx_assert(collision_groups.count(group_name)!=0,"Trying to get a collision group with name "<<group_name<<" that hasn't been built");
		return collision_groups[group_name];
	}
}
