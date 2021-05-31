#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
	class koules_t : public plant_t
	{
	public:
		koules_t(const std::string& path,int num_koules=4);
		virtual ~koules_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;
	protected:

		virtual void compute_derivative() override final;

		double x,y,theta,vx,vy;
		std::vector<std::vector<double>> koules_state;
		std::vector<std::vector<double>> koules_deriv;

		double vtheta,a;

		double ax,ay;

		int koules_size;
		double robot_radius;
		double koule_radius;

		space_point_t prev_state;
		space_point_t new_state;

	};
}
PRX_REGISTER_SYSTEM(koules_t, koules)
