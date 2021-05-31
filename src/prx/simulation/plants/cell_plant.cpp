
#include "prx/simulation/plants/cell_plant.hpp"

namespace prx
{

	cell_plant_t::cell_plant_t(const std::string& path) : plant_t(path)
	{
		x=y=0;
		state_memory = {&x,&y};
		state_space = new space_t("EE",state_memory,"XY");
		state_space->set_bounds({-100,-100},{100,100});

		v=theta=0;
		control_memory = {&v,&theta};
		input_control_space = new space_t("ER",control_memory,"VTheta");
		input_control_space->set_bounds({0,-3.14},{10,3.14});

		dx=dy=0;
		derivative_memory = {&dx,&dy};
		derivative_space = new space_t("EE",derivative_memory,"XdotYdot");

            geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->set_visualization_color("0x00ff00");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);
		
	}

	cell_plant_t::~cell_plant_t()
	{

	}

	void cell_plant_t::set_geo(double _x, double _y)
	{
		geometries["body"]->initialize_geometry({_x,_y,0.5});
		geometries["body"]->generate_collision_geometry();
	}

	void cell_plant_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
	}

	void cell_plant_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
	}

	void cell_plant_t::compute_derivative()
	{
		dx = v*cos(theta);
		dy = v*sin(theta);
	}
}
