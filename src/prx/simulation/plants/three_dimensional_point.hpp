#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
	class three_dimensional_point_t : public plant_t
	{
	public:
		three_dimensional_point_t(const std::string& path);
		virtual ~three_dimensional_point_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

	protected:

		virtual void compute_derivative() override final;

		double x,y,z,dx,dy,dz;

	};
}
PRX_REGISTER_SYSTEM(three_dimensional_point_t, 3D_Point)

