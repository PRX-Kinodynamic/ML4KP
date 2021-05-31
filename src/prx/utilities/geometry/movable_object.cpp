
#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
	std::vector<std::pair<std::string,std::weak_ptr<geometry_t>>> movable_object_t::get_geometries()
	{
		std::vector<std::pair<std::string,std::weak_ptr<geometry_t>>> pairs;
		for(auto&& g:geometries)
		{
			pairs.push_back(std::make_pair(object_name+"/"+g.first,std::weak_ptr<geometry_t>(g.second)));
		}
		return pairs;
	}

	std::vector<std::pair<std::string,std::weak_ptr<transform_t>>> movable_object_t::get_configurations()
	{
		std::vector<std::pair<std::string,std::weak_ptr<transform_t>>> pairs;
		for(auto&& t:configurations)
		{
			pairs.push_back(std::make_pair(object_name+"/"+t.first,std::weak_ptr<transform_t>(t.second)));
		}
		return pairs;
	}
}