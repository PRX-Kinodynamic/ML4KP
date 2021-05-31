#include "prx/simulation/integrators/runge_kutta4.hpp"

namespace prx
{
	void runge_kutta4_t::integrate(const double simulation_step) 
	{
		h = simulation_step == 0.0? h : simulation_step;
		if(!start_integration_state)
		{
			start_integration_state = state_space->make_point();
			k1 = derivative_space -> make_point();
			k2 = derivative_space -> make_point();
			k3 = derivative_space -> make_point();
			yn = state_space -> make_point();
		}
		state_space -> copy_to_point(start_integration_state);

		compute_derivative();
		// Compute k1 = h * f(X0, Y0)
		derivative_space -> copy_to_point(k1);
		// Compute d1 = Yn + k1 / 2
		state_space -> integrate(start_integration_state, derivative_space, h*0.5);

		compute_derivative();
		// Compute k2 = h * f(X0, Y0 + k1 / 2) = h * f(Xn + h / 2, d1)
		derivative_space -> copy_to_point(k2);
		// Compute d2 = Yn + k2 / 2 = Yn + h * 0.5 * f(Xn + h / 2, d1)
		state_space -> integrate(start_integration_state, derivative_space, h*0.5);

		compute_derivative();
		// Compute k3 = f(X0 + C3h, Y0 + k2 / 2) = h * f(Xn, h/2, d2)
		derivative_space -> copy_to_point(k3);
		// Compute d3 = Yn + k3 = Yn + h * f(Xn + h / 2, d2)
		state_space -> integrate(start_integration_state, derivative_space, h);

		// Compute k4 = h * f(Xn + h, Yn + k3) = h * f(Xn + h, d3)
		state_space -> copy_from_point(start_integration_state);
		compute_derivative();
		// Compute d4 = Yn + k4 / 6 = Yn + (h / 6) * f(Xn + h, d3)
		state_space -> integrate(derivative_space, h/6.0 );

		// Y_{n+1} = Yn + k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6 = d4 + k1 / 6 + k2 / 3 + k3 / 3
		state_space -> copy_to_point(yn);
		k1 -> multiply(h / 6.0);
		k2 -> multiply(h / 3.0);
		k3 -> multiply(h / 3.0);
		yn -> add(k1);
		yn -> add(k2);
		yn -> add(k3);
		
		state_space -> copy_from_point(yn);
	}
}