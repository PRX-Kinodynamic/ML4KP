#pragma once

#include "prx/utilities/geometry/geometry.hpp"

#include "prx/utilities/defs.hpp"

#include <unordered_map>

namespace prx
{
	class movable_object_t;

	template<class T>
	std::shared_ptr<movable_object_t> create_obstacle(T* new_obstacle)
	{
		std::shared_ptr<movable_object_t> new_ptr;
		new_ptr.reset(new_obstacle);
		return new_ptr;
	}

	class movable_object_t
	{
	public:
		movable_object_t(const std::string& o_name):object_name(o_name){}
		virtual ~movable_object_t(){}

		std::vector<std::pair<std::string,std::weak_ptr<geometry_t>>> get_geometries();
		std::vector<std::pair<std::string,std::weak_ptr<transform_t>>> get_configurations();

		inline std::string get_object_name()
		{
			return object_name;
		}
		
	protected:
		std::unordered_map<std::string,std::shared_ptr<geometry_t>> geometries;
		std::unordered_map<std::string,std::shared_ptr<transform_t>> configurations;
		std::string object_name;
	};
}
