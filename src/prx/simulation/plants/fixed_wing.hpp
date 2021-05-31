#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
	class fixed_wing_t : public plant_t
	{
	public:
		fixed_wing_t(const std::string& path);
		virtual ~fixed_wing_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;
                
	protected:

		virtual void compute_derivative() override final;


                
                const double m = 0.011; // mass
                const double wing_span = 0.28;
                const double wing_chord = 0.12;
                const double I_x = 0.33; // x moment of inertia
                const double I_y = 2.80; // x moment of inertia
                const double I_z = 3.13; // x moment of inertia
                const double N = 0.09; // maximum thrust
                const double max_specific_thrust = N / m;
                const double rho = 1.225; // density of air
                const double k = 1.86; // non dimensional constant k = ro*S/(2m)
                const double S = 2 * k * m / rho;
                const double g = 9.81;


                // position coordinates
                double _x;
                double _y;
                double _z;

                // Velocity
                double _V;     // flight speed
                double _gamma; // angle of attack
                double _chi;  // angle of sideslip
                // double _beta;  // angle of sideslip
                

                double _alpha;
                double _mu;
                double _thrust;

                // Euler angles
                // double _phi;     // roll
                // double _theta_p; // pitch  M3rcuri0V
                // double _psi;     // yaw

                // body axis angular rates
                // double _p;
                // double _q;
                // double _r;
                // double _flight;
                // double _thrust;

                // control inputs
                double T_c;     //thrust
                double alpha_c; // angle of attack
                double mu_c;      // wind axis roll angle


                // derivative state variables

                double _d_x;
                double _d_y;
                double _d_z;

                // Velocity
                double _d_V;     // flight speed
                double _d_gamma; // angle of attack
                double _d_chi; // angle of attack
                // double _d_beta;  // angle of sideslip
                
                // Euler angles
                // double _d_phi;     // roll
                // double _d_theta_p; // pitch
                // double _d_psi;     // yaw

                // body axis angular rates
                // double _d_p;
                // double _d_q;
                // double _d_r;

                double _d_thrust;
                double _d_alpha;
                double _d_mu;



                // double _des_thrust;
                // double _des_roll;
                // double _des_pitch;

                // double dx,dy,dz,droll,dpitch,dyaw,dv,dflight,dthrust;
                // double _v,_roll,_pitch,_yaw,_flight,_thrust;

	};
}
PRX_REGISTER_SYSTEM(fixed_wing_t, fixed_wing)
