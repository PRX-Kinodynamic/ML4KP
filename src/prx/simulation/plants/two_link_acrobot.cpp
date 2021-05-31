
#include "prx/simulation/plants/two_link_acrobot.hpp"

namespace prx
{

	two_link_acrobot_t::two_link_acrobot_t(const std::string& path) : plant_t(path)
	{
		_theta1=_theta2=_theta1dot=_theta2dot=0;
		state_memory = {&_theta1,&_theta2,&_theta1dot,&_theta2dot};
		state_space = new space_t("EEEE",state_memory,"TwoLinkState");
		state_space->set_bounds({0,-M_PI,-6,-6},{2.0*M_PI,M_PI,6,6});
		// state_space->set_bounds({-3.15,-3.15,-6,-6},{3.15,3.15,6,6});

		_tau=0;
		control_memory = {&_tau};
		input_control_space = new space_t("E",control_memory,"Torque");
		input_control_space->set_bounds({-7},{7});

		_theta1dotdot=_theta2dotdot=0;
		derivative_memory = {&_theta1dot,&_theta2dot,&_theta1dotdot,&_theta2dotdot};
		derivative_space = new space_t("EEEE",derivative_memory,"TwoLinkDeriv");

		const double length = 20;

		geometries["rod1"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["rod1"]->initialize_geometry({length,1,1});
		geometries["rod1"]->generate_collision_geometry();
		geometries["rod1"]->set_visualization_color("0x00ff00");
		configurations["rod1"]= std::make_shared<transform_t>();
		configurations["rod1"]->setIdentity();

		geometries["rod2"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["rod2"]->initialize_geometry({length,1,1});
		geometries["rod2"]->generate_collision_geometry();
		geometries["rod2"]->set_visualization_color("0xff0000");
		configurations["rod2"]= std::make_shared<transform_t>();
		configurations["rod2"]->setIdentity();

		geometries["ball"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["ball"]->initialize_geometry({1.5});
		geometries["ball"]->generate_collision_geometry();
		geometries["ball"]->set_visualization_color("0x0000ff");
		configurations["ball"]= std::make_shared<transform_t>();
		configurations["ball"]->setIdentity();

		// set_integrator("rk4");
		set_integrator(integrator_t::kRK4);

	}

	two_link_acrobot_t::~two_link_acrobot_t()
	{

	}

	void two_link_acrobot_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);

        _theta1 = norm_angle_pi(_theta1, 0, 2*M_PI);
        _theta2 = norm_angle_pi(_theta2, -M_PI, M_PI);
        state_space -> enforce_bounds();
	}

	void two_link_acrobot_t::update_configuration()
	{

		const double length = 20;
		auto body = configurations["rod2"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos((_theta1 - PRX_PI / 2) / 2.0), 0, 0, sin((_theta1 - PRX_PI / 2) / 2.0)).toRotationMatrix());
		body->translation() = (vector_t((length / 2.0) * cos(_theta1 - PRX_PI / 2), (length / 2.0) * sin(_theta1 - PRX_PI / 2), 1.5));

		body = configurations["rod1"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos((_theta1 + _theta2 - PRX_PI / 2) / 2.0),0, 0, sin((_theta1 + _theta2 - PRX_PI / 2) / 2.0)).toRotationMatrix());
		body->translation() = (vector_t((length) * cos(_theta1 - PRX_PI / 2)+(length / 2.0) * cos(_theta1 + _theta2 - PRX_PI / 2),
                                           (length) * sin(_theta1 - PRX_PI / 2)+(length / 2.0) * sin(_theta1 + _theta2 - PRX_PI / 2),
                                           1.5));

		body = configurations["ball"];
		body->setIdentity();
		body->translation() = (vector_t((length) * cos(_theta1 - PRX_PI / 2)+(length) * cos(_theta1 + _theta2 - PRX_PI / 2),
                                           (length) * sin(_theta1 - PRX_PI / 2)+(length) * sin(_theta1 + _theta2 - PRX_PI / 2),
                                           1.5));
	}

	void two_link_acrobot_t::compute_derivative()
	{
        const double theta2 = _theta2;
        const double theta1 = _theta1 - M_PI / 2.0;
        const double theta1dot = _theta1dot;
        const double theta2dot = _theta2dot;
        constexpr double I1 = 0.2;
        constexpr double I2 = 1.0;
        constexpr double l1 = 1.0;
        constexpr double l2 = 1.0;
        constexpr double lc1 = l1 / 2.0; 
        constexpr double lc2 = l2 / 2.0;
        constexpr double g = 9.81;
        constexpr double m = 1.0;
        constexpr double d1 = 1.0; // Damping
        constexpr double d2 = 1.0; // Damping

        //extra term m*lc2
        const double d11 = m * lc1 * lc1 + m * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc1 * cos(theta2)) + I1 + I2;
        const double d22 = m * lc2 * lc2 + I2;
        const double d12 = m * (lc2 * lc2 + l1 * lc2 * cos(theta2)) + I2;
        const double d21 = d12;

        //extra theta1dot
        const double c1 = -m * l1 * lc2 * theta2dot * theta2dot * sin(theta2) - (2.0 * m * l1 * lc2 * theta1dot * theta2dot * sin(theta2));
        const double c2 =  m * l1 * lc2 * theta1dot * theta1dot * sin(theta2);

        const double g1 = (m * lc1 + m * l1) * g * cos(theta1) + (m * lc2 * g * cos(theta1 + theta2));
        const double g2 = m * lc2 * g * cos(theta1 + theta2);

        const double u1 =  0;//    - 1.0 * d1 * theta1dot;
        const double u2 = _tau;// - 1.0 * d2 * theta2dot;
        const double theta1dot_dot = (d22 * (u1 - c1 - g1) - d12 * (u2 - c2 - g2)) / (d11 * d22 - d12 * d21);
        const double theta2dot_dot = (d11 * (u2 - c2 - g2) - d21 * (u1 - c1 - g1)) / (d11 * d22 - d12 * d21);

        _theta1dotdot = theta1dot_dot;
        _theta2dotdot = theta2dot_dot;
	}
}