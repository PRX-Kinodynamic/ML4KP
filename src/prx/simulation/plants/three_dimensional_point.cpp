
#include "prx/simulation/plants/three_dimensional_point.hpp"

namespace prx
{

	three_dimensional_point_t::three_dimensional_point_t(const std::string& path) : plant_t(path)
	{
		x=y=z=0;
		state_memory = {&x,&y,&z};
		state_space = new space_t("EEE",state_memory,"XYZ");
		state_space->set_bounds({-10,-10,-10},{10,10,10});

		dx=dy=dz=0;
		control_memory = {&dx,&dy,&dz};
		input_control_space = new space_t("EEE",control_memory,"XdotYdotZdot");
		input_control_space->set_bounds({-1,-1,-1},{1,1,1});

		derivative_memory = {&dx,&dy,&dz};
		derivative_space = new space_t("EEE",derivative_memory,"XdotYdotZdot");

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["body"]->initialize_geometry({.5});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0x00ff00");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();
		
		set_integrator(integrator_t::kEULER);

	}

	three_dimensional_point_t::~three_dimensional_point_t()
	{

	}

	void three_dimensional_point_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		//euler_integration(simulation_step);
	}

	void three_dimensional_point_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->translation() = (vector_t(x,y,z));
	}

	void three_dimensional_point_t::compute_derivative()
	{
	}
}