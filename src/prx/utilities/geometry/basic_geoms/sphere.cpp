#include "prx/utilities/geometry/basic_geoms/sphere.hpp"

namespace prx
{
	sphere_t::sphere_t(const std::string& object_name,double radius, const transform_t& pose) : movable_object_t(object_name)
	{
		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["body"]->initialize_geometry({radius});
		geometries["body"]->generate_collision_geometry();
		configurations["body"] = std::make_shared<transform_t>();
		*configurations["body"] = pose;
	}
}