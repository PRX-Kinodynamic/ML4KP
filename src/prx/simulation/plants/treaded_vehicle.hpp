#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
	/**
	 * @brief <b> A second-order differential drive vehicle. </b>
	 * 
	 * @author Zakary Littlefield, Aravind Sivaramakrishnan, Edgar Granados, Seth Karten
	 * */
	class treaded_vehicle_t : public plant_t
	{
	public:
		treaded_vehicle_t(const std::string& path);

		virtual ~treaded_vehicle_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		void set_cost_map(double ** cost_map, double delta_x, double delta_y, int , int , double);

		virtual void compute_stopping_maneuver(space_point_t, std::vector<double>&, std::vector<double>&) override final;


		virtual void compute_derivative() override final;
	protected:


		double x,y,theta,vl,vr,al,ar,dx,dy,dtheta;
		double **cost_map;
		int grid_height, grid_width;
		double delta_x, delta_y, cell_size;	// global to local space conversion for cost map
		bool use_cost_map;
	private:
		std::vector<double> lower_bound = {-11,-11,-3.15,-.7,-.7};
		std::vector<double> upper_bound = {11,11,3.15,.7,.7};
		friend system_factory_t;
	};


}
PRX_REGISTER_SYSTEM(treaded_vehicle_t, treaded_vehicle)

auto tv_vel_fn = [](prx::system_ptr_t sys_ptr)
{
    auto s = std::dynamic_pointer_cast<prx::treaded_vehicle_t>(sys_ptr);

	prx::space_point_t bk_state = s -> state_space -> make_point();
    prx::space_point_t bk_deriv = s -> derivative_space -> make_point();

	s -> state_space -> copy_to_point(bk_state);
    s -> derivative_space -> copy_to_point(bk_deriv);

	s -> state_space -> at(0) = s -> state_space -> at(1) = s -> state_space -> at(2) = 0;
	s -> state_space -> at(3) = s -> state_space -> get_bounds()[3].second;
	s -> state_space -> at(4) = s -> state_space -> get_bounds()[4].second;
            
    s -> compute_derivative();
    double vel = sqrt(std::pow(s -> state_space -> at(3), 2) + std::pow(s -> state_space -> at(4), 2));

    // Copy back the original values
	s -> state_space -> copy_from_point(bk_state);
	s -> derivative_space -> copy_from_point(bk_deriv);
	return vel;
};
PRX_REGISTER_VELOCITY_FN(treaded_vehicle, tv_vel_fn)
