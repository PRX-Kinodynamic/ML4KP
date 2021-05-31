#include "prx/utilities/geometry/basic_geoms/box.hpp"

namespace prx
{
	box_t::box_t(const std::string& object_name,double dim_x, double dim_y, double dim_z, const transform_t& pose) : movable_object_t(object_name)
	{
		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({dim_x,dim_y,dim_z});
		geometries["body"]->generate_collision_geometry();
		configurations["body"] = std::make_shared<transform_t>();
		*configurations["body"] = pose;
	}
}