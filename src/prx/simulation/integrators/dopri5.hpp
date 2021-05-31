#pragma once

#include "prx/simulation/integrators/integrator.hpp"


namespace prx
{

	class dopri5_t : public integrator_t
	{
	public:

		// atol : float or sequence absolute tolerance for solution
		// rtol : float or sequence relative tolerance for solution
		// nsteps : int Maximum number of (internally defined) steps allowed during one call to the solver.
		// first_step : float
		// max_step : float
		// safety : float Safety factor on new step selection (default 0.9)
		// ifactor : float
		// dfactor : float Maximum factor to increase/decrease step size by in one step
		// beta : float Beta parameter for stabilised step size control.

		dopri5_t(space_t* state_space, space_t* derivative_space, std::function<void ()> deriv_f,
			const double initial_simulation_step=0.01,
			const double _atol=1e-12,
			const double _rtol=1e-6, 
			const int _nsteps=500,
			const double _max_step=0.0, 
			const double _safety = 0.9,
			const double _ifactor=5.0,
			const double _dfactor=0.2, 
			const double _beta=0.0);
	
		dopri5_t(space_t* state_space, space_t* derivative_space, std::function<void ()> deriv_f, 
			const double initial_simulation_step, const std::vector<double> atol, const std::vector<double> rtol, 
			const int _nsteps=500,
			const double _max_step=0.0, 
			const double _safety = 0.9,
			const double _ifactor=10.0,
			const double _dfactor=0.2, 
			const double _beta=0.0);

		virtual ~dopri5_t();

		void integrate(const double simulation_step = 0.0) override;

		double error(const space_point_t& y1, const space_point_t& yh1 = nullptr);

        void set_Atol(double atol_sclr);
        void set_Atol(std::vector<double> atol_vec);

        void set_Rtol(double rtol_sclr);
        void set_Rtol(std::vector<double> rtol_vec);

	protected:

		void step();

		void initial_h();

	private:
		space_point_t Atol;
		space_point_t Rtol;
		double h_initial;
		int nstep;
		double max_step;
		double safety; // fac
		double facmin;
		double facmax;
		double beta;

		space_point_t yn;
		space_point_t yhn;
		space_point_t k1;
		space_point_t k2;
		space_point_t k3;
		space_point_t k4;
		space_point_t k5;
		space_point_t k6;
		space_point_t k7;
		space_point_t k_cache;

		static constexpr double a21 = 1.0/5.0;
		
		static constexpr double a31 = 3.0/40.0;
		static constexpr double a32 = 9.0/40.0;
		
		static constexpr double a41 = 44.0/45.0;
		static constexpr double a42 = -56.0/15.0;
		static constexpr double a43 = 32.0/9.0;

		static constexpr double a51 = 19372.0/6561.0;
		static constexpr double a52 = -25360.0/2187.0;
		static constexpr double a53 = 64448.0/6561.0;
		static constexpr double a54 = -212.0/729.0;

		static constexpr double a61 = 9017.0/3168.0;
		static constexpr double a62 = -355.0/33.0;
		static constexpr double a63 = 46732.0/5247.0;
		static constexpr double a64 = 49.0/176.0;
		static constexpr double a65 = -5103.0/18656.0;

		static constexpr double a71 = 35.0/384.0;
		static constexpr double a72 = 0;
		static constexpr double a73 = 500.0/1113.0;
		static constexpr double a74 = 125.0/192.0;
		static constexpr double a75 = -2187.0/6784.0;
		static constexpr double a76 = 11.0/84.0;
	
		static constexpr double b1 = a71;
		static constexpr double b2 = a72;
		static constexpr double b3 = a73;
		static constexpr double b4 = a74;
		static constexpr double b5 = a75;
		static constexpr double b6 = a76;
		static constexpr double b7 = 0;

		static constexpr double bh1 = 5179.0/57600.0;
		static constexpr double bh2 = 0;
		static constexpr double bh3 = 7571.0/16695.0;
		static constexpr double bh4 = 393.0/640.0;
		static constexpr double bh5 = -92097.0/339200.0;
		static constexpr double bh6 = 187.0/2100.0;
		static constexpr double bh7 = 1.0/40.0;
	};
}
