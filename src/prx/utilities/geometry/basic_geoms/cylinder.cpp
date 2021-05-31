#include "prx/utilities/geometry/basic_geoms/cylinder.hpp"

namespace prx
{
	cylinder_t::cylinder_t(const std::string& object_name,double radius,double height, const transform_t& pose) : movable_object_t(object_name)
	{
		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["body"]->initialize_geometry({radius,height});
		geometries["body"]->generate_collision_geometry();
		configurations["body"] = std::make_shared<transform_t>();
		*configurations["body"] = pose;
	}
}