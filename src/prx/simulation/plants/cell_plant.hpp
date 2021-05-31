#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
	class cell_plant_t : public plant_t
	{
	public:
		cell_plant_t(const std::string& path);
		virtual ~cell_plant_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;
		virtual void set_geo(double, double) override;
	protected:

		virtual void compute_derivative() override final;

		double x,y,v,theta,dx,dy;

	};
}
