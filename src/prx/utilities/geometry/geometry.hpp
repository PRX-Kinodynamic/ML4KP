#pragma once

#include "prx/utilities/defs.hpp"

#include "prx/external/PQP/PQP.h"

#include <memory>

namespace prx
{
	enum class geometry_type_t
	{
		//initialized with lengths on each axis
		BOX = 0,
		//initialized with radius
		SPHERE = 1,
		//initialized with lengths along each axis
		ELLIPSOID = 2,
		//initialized with radius of caps and length of cylinder
		CAPSULE = 3,
		//initialed with radius of cone opening and height of cone
		CONE = 4,
		//initialized with radius of cylinder and height
		CYLINDER = 5
	};

	class geometry_t
	{
	public:

		geometry_t(geometry_type_t new_geom_type);
		~geometry_t();

		std::weak_ptr<PQP_Model> get_collision_geometry();

		geometry_type_t get_geometry_type();

		std::vector<double> get_geometry_params();

		void initialize_geometry(const std::vector<double>& geom_params);

		void generate_collision_geometry();

		void set_visualization_color(std::string c)
		{
			vis_color = c;
		}

		std::string get_visualization_color()
		{
			return vis_color;
		}

	private:

		std::shared_ptr<PQP_Model> collision_geometry;

		std::string vis_color;

		geometry_type_t geom_type;
		std::vector<double> params;
		bool params_set;

	};
}