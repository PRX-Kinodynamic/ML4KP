#pragma once

#include "prx/simulation/plant.hpp"
// #include "prx/simulation/system_factory.hpp"

namespace prx
{

	class two_dimensional_point_t : public plant_t
	{
	public:
		two_dimensional_point_t(const std::string& path);
		virtual ~two_dimensional_point_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		virtual void set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper) override final;
		// void set_bounds();
	protected:

		virtual void compute_derivative() override final;

		double x,y,v,theta,dx,dy;
	private:

	};

}
PRX_REGISTER_SYSTEM(two_dimensional_point_t, 2D_Point)
PRX_REGISTER_VELOCITY_FN(2D_Point, [](system_ptr_t s){return s -> input_control_space -> get_bounds()[0].second;})