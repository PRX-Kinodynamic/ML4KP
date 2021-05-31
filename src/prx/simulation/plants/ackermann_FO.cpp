
#include "prx/simulation/plants/ackermann_FO.hpp"

namespace prx
{

	ackermann_FO::ackermann_FO(const std::string& path) : plant_t(path)
	{
		x = y = theta = 0.0;
		state_memory = {&x, &y, &theta};
		state_space = new space_t("EEE",state_memory,"ackermann_FO_ss");
		state_space -> set_bounds(min_state_bound, max_state_bound);

		gamma = V =0;
		control_memory = {&gamma, &V};
		input_control_space = new space_t("EE",control_memory,"ackermann_FO_cs");
		input_control_space -> set_bounds(min_ctrl_bound, max_ctrl_bound);

		x_dot = y_dot = theta_dot = 0.0;
		derivative_memory = {&x_dot, &y_dot, &theta_dot};
		derivative_space = new space_t("EEE",derivative_memory,"ackermann_FO_ds");

		const double length = 2;

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"] -> initialize_geometry({length,1,1});
		geometries["body"] -> generate_collision_geometry();
		geometries["body"] -> set_visualization_color("0x00ff00");
		configurations["body"] = std::make_shared<transform_t>();
		configurations["body"] -> setIdentity();

		set_integrator(integrator_t::kRK4);

	}

	ackermann_FO::~ackermann_FO()
	{

	}

	void ackermann_FO::propagate(const double simulation_step)
	{		
		integrator -> integrate(simulation_step);
        state_space -> enforce_bounds();
	}

	void ackermann_FO::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2.),0,0,sin(theta/2.)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0.5));
	}

	void ackermann_FO::compute_derivative()
	{
        x_dot = V * std::cos(theta);
        y_dot = V * std::sin(theta);
        theta_dot = (V / L) * std::tan(gamma);

	}

}