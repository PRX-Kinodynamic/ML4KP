#pragma once
#include "prx/utilities/spaces/space.hpp"


namespace prx
{

	typedef std::function<void ()> compute_derivative_t;
	/**
	 * @file integrator.hpp
	 * @brief <b>A base class for different integrators.</b>
	 * 
	 * Given the ODE for a plant of the form \dot{x} = f(x,u), the integrator integrates the forward dynamics 
	 * given a simulation step size.
	 * 
	 * @authors Edgar Granados
	 */
	class integrator_t
	{
	public:
		integrator_t(space_t* _state_space, space_t* _derivative_space, std::function<void ()> deriv_f )
		{
			h = 0.01;
			state_space = _state_space;
			derivative_space = _derivative_space;
			compute_derivative = deriv_f;
		}
		integrator_t(space_t* _state_space, space_t* _derivative_space, std::function<void ()> deriv_f, const double initial_simulation_step)
		{
			h = initial_simulation_step;
			state_space = _state_space;
			derivative_space = _derivative_space;
			compute_derivative = deriv_f;
		}
		~integrator_t() {};

		virtual void integrate(const double simulation_step = 0.0) = 0;

		enum integrators {kEULER=0, kRK4=1, kDOPRI5=2};

	protected:
		space_point_t start_integration_state;
		double h;
		compute_derivative_t compute_derivative;
		space_t* state_space;
		space_t* derivative_space;		
	private:

	};
}
