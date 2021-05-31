#pragma once

#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include "prx/simulation/collision_checking/collision_group.hpp"

#include <vector>

namespace prx
{
	class bullet_collision_group_t : public collision_group_t
	{
	public:
	  	bullet_collision_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles={});
		~bullet_collision_group_t();

		bool in_collision() override;

		std::vector<std::shared_ptr<bullet_t> > bullet_plants;
		// bool use_bullet;

	};
}
