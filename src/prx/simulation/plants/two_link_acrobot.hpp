#pragma once

#include "prx/simulation/plant.hpp"


namespace prx
{
	class two_link_acrobot_t : public plant_t
	{
	public:
		two_link_acrobot_t(const std::string& path);
		virtual ~two_link_acrobot_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

	protected:

		virtual void compute_derivative() override final;

		double _theta1,_theta2,_theta1dot,_theta2dot,_tau,_theta1dotdot,_theta2dotdot;

	};
}

PRX_REGISTER_SYSTEM(two_link_acrobot_t, Acrobot)
