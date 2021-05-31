
#include "prx/simulation/plant.hpp"
#include "prx/utilities/defs.hpp"


namespace prx
{

	plant_t::plant_t(const std::string& path) : system_t(path),movable_object_t(path)
	{
		derivative_space=nullptr;

		collision_list.clear();
		set_integrator(integrator_t::kEULER);
	}

	plant_t::~plant_t()
	{
		if(derivative_space!=nullptr)
		{
			delete derivative_space;
		}
	}

	void plant_t::set_integrator(integrator_t::integrators integrator_choice)
	{
		double initial_integration_step = simulation_step;
		//printf("simulation_step: %.6f\n", simulation_step);
		std::function<void ()> deriv_f = [this](){this -> compute_derivative();};
		if (integrator_choice == integrator_t::kEULER) 
			integrator = std::make_shared<euler_t>(state_space, derivative_space, deriv_f, initial_integration_step);
		else if (integrator_choice == integrator_t::kRK4) 
			integrator = std::make_shared<runge_kutta4_t>(state_space, derivative_space, deriv_f, initial_integration_step);
		else if (integrator_choice == integrator_t::kDOPRI5) 
			integrator = std::make_shared<dopri5_t>(state_space, derivative_space, deriv_f, initial_integration_step);
		else printf("Invalid integrator choice. Available integrators are: euler, rk4 and dopri5\n");

	}	
    
	void plant_t::add_system(system_ptr_t& ptr)
	{
		prx_throw("Trying to add a system ("<<ptr->get_pathname()<<") into a plant ("<<pathname<<")");
	}

	void plant_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		//euler_integration(simulation_step);
	}

	void plant_t::compute_control()
	{
		//the control should have already been set in the control space
	}

	void plant_t::compute_stopping_maneuver(space_point_t start_state, std::vector<double>& times, std::vector<double>& ctrls)
	{
		//the control should have already been set in the vehicle space
	}
	
	void plant_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
	{
		state_space->set_bounds(lower, upper);
	}

}
