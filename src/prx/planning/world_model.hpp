#pragma once

#include "prx/simulation/system_group_manager.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include <unordered_map>

namespace prx
{
	typedef std::pair<std::shared_ptr<system_group_t>,std::shared_ptr<collision_group_t>> world_model_context;
	
	template<
    class SGM = system_group_manager_t,
    class CC  = collision_checker_t
    > 
	class world_model_t
	{
	public:
		
		// template<typename SGM = system_group_manager_t, typename CC = collision_checker_t> 
		explicit world_model_t(const std::vector<system_ptr_t>& all_systems,const std::vector<std::shared_ptr<movable_object_t>>& all_obstacles)
		{
			system_groups = new SGM();
			collision_groups = new CC();

			for(auto s : all_systems)
			{
				systems[s->get_pathname()] = s;
			}
			for(auto o : all_obstacles)
			{
				obstacles[o->get_object_name()] = o;
			}
		}

		// world_model_t(const std::vector<system_ptr_t>& all_systems,const std::vector<std::shared_ptr<movable_object_t>>& all_obstacles) 
		// 	: world_model_t<system_group_manager_t, collision_checker_t>(all_systems, all_obstacles)
		// 	{};
		
		~world_model_t()
		{
			delete system_groups;
			delete collision_groups;
		};

		inline world_model_context get_context(const std::string& context_name)
		{
			return std::make_pair(system_groups->get_system_group(context_name),collision_groups->get_collision_group(context_name));
		}

		void create_context(const std::string& context_name, 
			const std::vector<std::string>& system_names, const std::vector<std::string>& obstacle_names)
		{
			all_context_names.push_back(context_name);
			std::vector<system_ptr_t> context_systems;
			std::vector<std::shared_ptr<movable_object_t>> context_obstacles;
			for(auto&& s : system_names)
			{
				context_systems.push_back(systems[s]);
			}
			for(auto&& o : obstacle_names)
			{
				context_obstacles.push_back(obstacles[o]);
			}

			system_groups->add_system_group(context_name,context_systems);
			collision_groups->add_collision_group(context_name,context_systems,context_obstacles);
		}

		inline std::vector<std::string> get_all_context_names()
		{
			return all_context_names;
		}

	private:
		SGM* system_groups;
		CC* collision_groups;

		std::unordered_map<std::string,system_ptr_t> systems;
		std::unordered_map<std::string,std::shared_ptr<movable_object_t>> obstacles;

		std::vector<std::string> all_context_names;
	};
}