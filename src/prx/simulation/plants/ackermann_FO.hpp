#pragma once

#include "prx/simulation/plant.hpp"

#include <Eigen/Dense>
#include <Eigen/Core>

namespace prx
{
	class ackermann_FO : public plant_t
	{
	public:
		ackermann_FO(const std::string& path);
		virtual ~ackermann_FO();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

	protected:

		virtual void compute_derivative() override final;

		double x;
		double y;
		double theta;

		double x_dot;
		double y_dot;
		double theta_dot;

		double gamma;
		double V;

		// Distance between front and back wheels
		double L = 1.5; 

		const double max_delta_deg = 60;
		const double max_delta_rad = max_delta_deg * PRX_PI / 180.0;

		std::vector<double> min_ctrl_bound = {-max_delta_rad, -30};
		std::vector<double> max_ctrl_bound = {max_delta_rad, 30};

		std::vector<double> min_state_bound = {-10, -10, -PRX_PI};
		std::vector<double> max_state_bound = { 10,  10, PRX_PI};
	};
}

PRX_REGISTER_SYSTEM(ackermann_FO, Ackermann_FO)
PRX_REGISTER_VELOCITY_FN(Ackermann_FO, [](system_ptr_t s){return s -> input_control_space -> get_bounds()[1].second;})

