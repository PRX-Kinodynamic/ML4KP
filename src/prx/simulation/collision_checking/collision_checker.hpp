#pragma once

#include "prx/simulation/plant.hpp"
// #ifndef BULLET_NOT_BUILT
// #include "prx/bullet_sim/plants/bullet_plant.hpp"
// #endif
#include "prx/simulation/collision_checking/collision_group.hpp"

#include "prx/utilities/defs.hpp"

#include <unordered_map>

namespace prx
{

	class collision_checker_t
	{
	public:
		collision_checker_t();
		virtual ~collision_checker_t();

		virtual void add_collision_group(const std::string& group_name, const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles);
	  
		std::shared_ptr<collision_group_t> get_collision_group(const std::string& group_name);

	protected:
		std::unordered_map<std::string,std::shared_ptr<collision_group_t>> collision_groups;
	};
}
