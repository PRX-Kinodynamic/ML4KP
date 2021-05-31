#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
    class treaded_vehicle_first_order_t : public plant_t
	{
	public:
		treaded_vehicle_first_order_t(const std::string& path);

		virtual ~treaded_vehicle_first_order_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		void set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper) override;

		virtual void compute_derivative() override final;
	protected:


		double x,y,theta,vl,vr,dx,dy,dtheta;

		std::vector<double> lower_bound = {-11,-11,-3.15};
		std::vector<double> upper_bound = {11,11,3.15};
	};
}
PRX_REGISTER_SYSTEM(treaded_vehicle_first_order_t, FO_treaded_vehicle)

auto tvfo_vel_fn = [](prx::system_ptr_t sys_ptr)
{
    auto tv_fo = std::dynamic_pointer_cast<prx::treaded_vehicle_first_order_t>(sys_ptr);
    if (!tv_fo) return std::numeric_limits<double>::infinity();
	prx::space_point_t bk_state = tv_fo -> state_space -> make_point();
    prx::space_point_t bk_deriv = tv_fo -> derivative_space -> make_point();

    tv_fo -> state_space -> copy_to_point(bk_state);
    tv_fo -> derivative_space -> copy_to_point(bk_deriv);

    tv_fo -> state_space -> at(0) = tv_fo -> state_space -> at(1) = tv_fo -> state_space -> at(2) = 0;
    tv_fo -> input_control_space -> at(0) = tv_fo -> input_control_space -> get_bounds()[0].second;
    tv_fo -> input_control_space -> at(1) = tv_fo -> input_control_space -> get_bounds()[1].second;
            
    tv_fo -> compute_derivative();
    double vel = sqrt(std::pow(tv_fo -> derivative_space -> at(0), 2) + std::pow(tv_fo -> derivative_space -> at(1), 2));

	// Copy back the original values
	tv_fo -> state_space -> copy_from_point(bk_state);
	tv_fo -> derivative_space -> copy_from_point(bk_deriv);
	return vel;
};
PRX_REGISTER_VELOCITY_FN(FO_treaded_vehicle, tvfo_vel_fn)
