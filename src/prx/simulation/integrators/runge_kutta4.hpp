#pragma once

#include "prx/simulation/integrators/integrator.hpp"


namespace prx
{

	class runge_kutta4_t : public integrator_t
	{
	public:
		runge_kutta4_t(space_t* state_space, space_t* derivative_space, std::function<void ()> deriv_f) : 
			integrator_t(state_space, derivative_space, deriv_f) {}

		runge_kutta4_t(space_t* state_space, space_t* derivative_space, std::function<void ()> deriv_f, const double initial_simulation_step) : 
			integrator_t(state_space, derivative_space, deriv_f, initial_simulation_step) {}

		~runge_kutta4_t() {};

		void integrate(const double simulation_step = 0.0) override;

	protected:
		space_point_t yn;
		space_point_t k1;
		space_point_t k2;
		space_point_t k3;
	private:
	};
}
