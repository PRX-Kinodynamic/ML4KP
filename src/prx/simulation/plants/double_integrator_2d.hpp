#pragma once

#include "prx/simulation/plant.hpp"

namespace prx
{
    class double_integrator_2d_t : public plant_t 
    {
        public:
        double_integrator_2d_t(const std::string& path);
        virtual ~double_integrator_2d_t();

        virtual void propagate(const double simulation_step) override final;

		virtual void update_configuration() override;

        protected:
        virtual void compute_derivative() override final;

		double x,y,dx,dy,ddx,ddy;
    };
}
PRX_REGISTER_SYSTEM(double_integrator_2d_t, 2D_DoubleInt)