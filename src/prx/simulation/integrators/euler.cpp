#include "prx/simulation/integrators/euler.hpp"

namespace prx
{
	euler_t::euler_t(space_t* state_space, space_t* derivative_space, std::function<void ()>  deriv_f) : 
		integrator_t(state_space, derivative_space, deriv_f) 
	{
		start_integration_state = nullptr;
	}

	euler_t::euler_t(space_t* state_space, space_t* derivative_space, std::function<void ()> deriv_f, const double initial_simulation_step) : 
			integrator_t(state_space, derivative_space, deriv_f, initial_simulation_step) 
	{
		start_integration_state = nullptr;
	}

	euler_t::~euler_t()
	{

	}


	void euler_t::integrate(const double simulation_step) 
	{
		compute_derivative();
		state_space -> integrate(derivative_space, simulation_step);
	}
}