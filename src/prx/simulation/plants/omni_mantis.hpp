#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
	class omni_mantis_t : public plant_t
	{
	public:
		omni_mantis_t(const std::string& path);
		virtual ~omni_mantis_t();

		virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;
	protected:

		virtual void compute_derivative() override final;

        double _x;
        double _y;
        double _z;
        double _theta;
        double _vx;
        double _vy;
        double _vz;
        double _vtheta;
        double _ax;
        double _ay;
        double _az;
        double _atheta;

        double _dx;
        double _dy;


		std::vector<std::shared_ptr<transform_t>> transforms;

	};
}
PRX_REGISTER_SYSTEM(omni_mantis_t, omni_mantis)
