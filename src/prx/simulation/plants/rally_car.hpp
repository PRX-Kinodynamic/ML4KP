#pragma once

#include "prx/simulation/plant.hpp"

// Taken from:
//	 ==> Velenis, E., Tsiotras, P., & Lu, J. (2008). 
//		 Optimality properties and driver input parameterization 
//		 for trail-braking cornering. European Journal of Control, 
//		 14(4), 308-320.
//	 ==> Velenis, Efstathios. "Analysis and control of high-speed
//		 wheeled vehicles." PhD diss., Georgia Institute of Technology,
//		 2006.
// For more details of this system refer to: https://www.sciencedirect.com/science/article/pii/S0947358008707751
namespace prx
{
	class rally_car_t : public plant_t
	{
	public:
		rally_car_t(const std::string& path);
		virtual ~rally_car_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

	protected:

		virtual void compute_derivative() override final;

		// double x, y, theta;
		// double dx, dy, d_theta;
		// double v_linear, v_angular;

		// state 
		double _x;
        double _y;
        double _vx;
        double _vy;
        double _theta;
        double _thetadot;
        double _wf;
        double _wr;

        // control 
        double _sta;
        double _tf;
        double _tr;

        // derivatives
        double d_x;
        double d_y;
        double d_vx;
        double d_vy;
        double d_theta;
        double d_thetadot;
        double d_wf;
        double d_wr;

        const double M = 145;
		const double IZ = 274;
		const double LF = 1.3;
		const double LR = 1.4;
		const double R = .3;
		const double IF = 1.8 ;
		const double IR = 1.8;
		const double H = .4    ;
		const double B = 7;
		const double C = 1.6;
		const double D = .52;

	};
}

PRX_REGISTER_SYSTEM(rally_car_t, rally_car)
auto rc_vel_fn = [](prx::system_ptr_t s)
{
    return std::sqrt(std::pow(s -> state_space -> get_bounds()[3].second, 2) + std::pow(s -> state_space -> get_bounds()[4].second, 2));
};
PRX_REGISTER_VELOCITY_FN(rally_car, rc_vel_fn)