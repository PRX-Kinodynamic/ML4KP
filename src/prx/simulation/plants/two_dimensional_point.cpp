
#include "prx/simulation/plants/two_dimensional_point.hpp"

namespace prx
{
	//bool two_dimensional_point_t::s_registered =
	//	system_factory_t::register_plant(PLANT_NAME, //&create_2dpt);
	//		[](const std::string path) -> plant_ptr_t{return std::make_shared<two_dimensional_point_t>(path);});
	

	two_dimensional_point_t::two_dimensional_point_t(const std::string& path) : plant_t(path)
	{
		// s_registered;
		x=y=0;
		state_memory = {&x,&y};
		state_space = new space_t("EE",state_memory,"XY");
		state_space->set_bounds(
			{-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max()},
			{ std::numeric_limits<double>::max(), std::numeric_limits<double>::max()});

		v=theta=0;
		control_memory = {&v,&theta};
		input_control_space = new space_t("ER",control_memory,"VTheta");
		input_control_space->set_bounds({0.3,-3.14},{1,3.14});

		dx=dy=0;
		derivative_memory = {&dx,&dy};
		derivative_space = new space_t("EE",derivative_memory,"XdotYdot");

		const std::string shape = "cylinder";
		const double tdpt_size = 0.5;
		if(shape=="sphere")
		{
			geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
			geometries["body"]->initialize_geometry({tdpt_size});
		}
		if(shape=="box")
		{
			geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
			geometries["body"]->initialize_geometry({tdpt_size, tdpt_size, tdpt_size});
		}
		if(shape=="cylinder")
		{
			geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
			geometries["body"]->initialize_geometry({tdpt_size,.5});
		}
		if(shape=="capsule")
		{
			geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
			geometries["body"]->initialize_geometry({tdpt_size,1});
		}
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0x00ff00");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);

	}

	two_dimensional_point_t::~two_dimensional_point_t()
	{

	}
	
	void two_dimensional_point_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
	{
		state_space->set_bounds(lower, upper);
	}

	void two_dimensional_point_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		
	}

	void two_dimensional_point_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
	}

	void two_dimensional_point_t::compute_derivative()
	{
		dx = v*cos(theta);
		dy = v*sin(theta);
	}
}