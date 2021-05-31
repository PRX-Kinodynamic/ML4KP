#pragma once

#include "prx/simulation/plant.hpp"

// Taken from:
//	 @INPROCEEDINGS{7577009,
// 		author={A. {Arab} and K. {Yu} and J. {Yi} and Y. {Liu}},
// 		booktitle={2016 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)}, 
// 		title={Motion control of autonomous aggressive vehicle maneuvers}, 
// 		year={2016},
// 		volume={},
// 		number={},
// 		pages={1663-1668},}
namespace prx
{
	class racecar_mini_t : public plant_t
	{
	public:
		racecar_mini_t(const std::string& path);
		virtual ~racecar_mini_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

		void set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper) override;

	protected:

		virtual void compute_derivative() override final;

		// double x, y, theta;
		// double dx, dy, d_theta;
		// double v_linear, v_angular;

		// state 
		double _x;
        double _y;
        double _xdot;
        double _ydot;
        double _psi;
        double _psidot;
        double _wFr;
        double _wFl;
        double _wRr;
        double _wRl;

        // control 
        double _delta;
        double _Fflx;
        double _Ffrx;
        double _Frlx;
        double _Frrx;

        // derivatives
        //double d_x;
        //double d_y;
        double _xdotdot;
        double _ydotdot;
        //double d_theta;
        double _psidotdot;
        double _wFrdot;
        double _wFldot;
        double _wRrdot;
        double _wRldot;

        static constexpr double g = 9.81;

        // mini!?
        static constexpr double M = 6.0;
		static constexpr double IZ = 0.25;
		static constexpr double L1 = 0.2;
		static constexpr double L2 = 0.2;
		static constexpr double W = 0.15;
		static constexpr double R = 0.025;
		//static constexpr double IF = 1.8;
		//static constexpr double IR = 1.8;
		static constexpr double H = .05;
		static constexpr double B = 2;
		static constexpr double C = 0.6;
		static constexpr double D = 0.05;

		static constexpr double L = L1 + L2;
		static constexpr double d_SV = 1.;

		std::vector<double> lower_bound = {-26, -24, -3.14159, -5, -5, -0.5};
		std::vector<double> upper_bound = {  0,  24,  3.14159,  5,  5,  0.5};

	};
}
PRX_REGISTER_SYSTEM(racecar_mini_t, racecar_mini)
