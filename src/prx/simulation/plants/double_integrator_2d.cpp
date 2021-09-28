#include "prx/simulation/plants/double_integrator_2d.hpp"

namespace prx
{
    double_integrator_2d_t::double_integrator_2d_t(const std::string& path) : plant_t(path)
    {
        x=y=dx=dy=0;
        state_memory = {&x, &y, &dx, &dy};
        state_space = new space_t("EEEE",state_memory,"XYdXdY");
        state_space->set_bounds(
            {-10.0,-10.0,-1.0,-1.0},
            { 10.0, 10.0, 1.0, 1.0}
            );

        ddx=ddy=0;
        control_memory = {&ddx,&ddy};
        input_control_space = new space_t("EE",control_memory,"ddX,ddY");
        input_control_space->set_bounds(
            {-0.2, -0.2}, {0.2, 0.2}
            );
        
        derivative_memory = {&dx,&dy,&ddx,&ddy};
        derivative_space = new space_t("EEEE",derivative_memory,"dXdYddXddY");

        geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
        geometries["body"]->initialize_geometry({0.1});
        geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0x00ff00");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);
    }

    double_integrator_2d_t::~double_integrator_2d_t()
    {

    }

    void double_integrator_2d_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		
	}

    void double_integrator_2d_t::update_configuration()
    {
        auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(0,0,0,1).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
    }

    void double_integrator_2d_t::compute_derivative()
    {
        
    }
}