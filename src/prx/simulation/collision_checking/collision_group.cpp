
#include "prx/simulation/collision_checking/collision_group.hpp"

namespace prx
{
	void collision_group_t::pqp_info_t::update_info()
	{
		auto config_ptr = transform.lock();
		pos[0] = config_ptr->translation()(0);
		pos[1] = config_ptr->translation()(1);
		pos[2] = config_ptr->translation()(2);
        rot[0][0] = config_ptr->linear()(0,0);
        rot[0][1] = config_ptr->linear()(0,1);
        rot[0][2] = config_ptr->linear()(0,2);
        rot[1][0] = config_ptr->linear()(1,0);
        rot[1][1] = config_ptr->linear()(1,1);
        rot[1][2] = config_ptr->linear()(1,2);
        rot[2][0] = config_ptr->linear()(2,0);
        rot[2][1] = config_ptr->linear()(2,1);
        rot[2][2] = config_ptr->linear()(2,2);
	}

  	collision_group_t::collision_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles)
	{
		for(auto&& object: in_obstacles)
		{
			auto geoms = object->get_geometries();
			auto configs = object->get_configurations();
			for(int i=0;i<geoms.size();i++)
			{
				prx_assert(geoms[i].first==configs[i].first,"Geometry and configuration lists don't match in "<<object->get_object_name());
				auto g = geoms[i].second;
				auto config = configs[i].second;
				auto g_ptr = g.lock();
				auto info = std::make_shared<pqp_info_t>();
				info->model = g_ptr->get_collision_geometry();
				info->transform = config;
				infos.push_back(info);
			}
		}
		
		for(auto&& sys: in_plants)
		{
			auto plant = std::dynamic_pointer_cast<plant_t>(sys);
			if(plant)
			{
			  // #ifndef BULLET_NOT_BUILT
			  // auto plant_bullet = std::dynamic_pointer_cast<bullet_t>(sys);
			  // if(plant_bullet){
			  //   bullet_plants.push_back(plant_bullet);
			  // }
			  // #endif
     
				std::vector<std::shared_ptr<pqp_info_t>> local_infos;
				plants.push_back(sys);
				auto collision_list = plant->get_collision_list();
				auto geoms = plant->get_geometries();
				auto configs = plant->get_configurations();
				for(int i=0;i<geoms.size();i++)
				{
					prx_assert(geoms[i].first==configs[i].first,"Geometry and configuration lists don't match in "<<plant->get_object_name());
					auto g = geoms[i].second;
					auto config = configs[i].second;
					auto g_ptr = g.lock();
					auto info = std::make_shared<pqp_info_t>();
					info->model = g_ptr->get_collision_geometry();
					info->transform = config;
					local_infos.push_back(info);
				}

				for(auto prev_info : infos)
				{
					for(auto local_info : local_infos)
					{
						collision_cache.push_back(std::make_pair(prev_info,local_info));
					}
				}

				for(auto collision_pair : collision_list)
				{
					collision_cache.push_back(std::make_pair(local_infos[collision_pair.first],
														local_infos[collision_pair.second]));
				}

				for(auto local_info : local_infos)
				{
					infos.push_back(local_info);
				}

			}
		}
	}

	collision_group_t::~collision_group_t()
	{
	}

	bool collision_group_t::in_collision()	  
	{	
		update_plants();
		
        for(auto&& cc_pair: collision_cache)
        {
        	auto info1 = cc_pair.first.lock();
        	auto info2 = cc_pair.second.lock();
	        PQP_CollideResult collision_result;
	        PQP_Collide(&collision_result,
	                    info1->rot, info1->pos, info1->model.lock().get(),
	                    info2->rot, info2->pos, info2->model.lock().get(),
	                    PQP_FIRST_CONTACT);
	        if(collision_result.Colliding())
	        	return true;
	    }
	    return false;
	}

	collision_group_t::pqp_distance_t collision_group_t::get_distances()
	{
		pqp_distance_t result;
		update_plants();

		for (auto&& cc_pair : collision_cache)
		{
			auto info2 = cc_pair.first.lock();
			auto info1 = cc_pair.second.lock();
			PQP_DistanceResult distance_result;
			PQP_Distance(&distance_result,
				info1->rot, info1->pos, info1->model.lock().get(),
				info2->rot, info2->pos, info2->model.lock().get(),
				0.0,0.0);
			result.distances.push_back(distance_result.Distance());
			auto p1 = distance_result.P1();
			std::vector<double> closest_point;
			closest_point.push_back(info1->pos[0]+p1[0]);
			closest_point.push_back(info1->pos[1]+p1[1]);
			result.closest_points.push_back(closest_point);
		}
		return result;
	}

	void collision_group_t::update_plants()
	{
		for(auto&& sys: plants)
		{
			auto plant = std::dynamic_pointer_cast<plant_t>(sys);
			plant->update_configuration();
		}

		for(auto&& element: infos)
		{
			element->update_info();
		}
	}
}
