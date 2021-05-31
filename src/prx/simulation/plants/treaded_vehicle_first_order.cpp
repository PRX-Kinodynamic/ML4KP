#include "prx/simulation/plants/treaded_vehicle_first_order.hpp"

namespace prx
{
    treaded_vehicle_first_order_t::treaded_vehicle_first_order_t(const std::string& path) : plant_t(path)
    {
        x=y=theta=0;
        state_memory = {&x,&y,&theta};
		state_space = new space_t("EER",state_memory,"TreadedFOState");
		state_space->set_bounds({-11,-11,-3.15},{11,11,3.15});

        vl=vr=0;
        control_memory = {&vl,&vr};
        input_control_space = new space_t("EE",control_memory,"TreadedFOControl");
        input_control_space->set_bounds({-0.7,-0.7},{0.7,0.7});

        dx=dy=dtheta=0;
        derivative_memory = {&dx,&dy,&dtheta};
        derivative_space = new space_t("EEE",derivative_memory,"ThetaFODerivative");

        geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.9,.6,.25});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0xff00ff");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);

    }

    treaded_vehicle_first_order_t::~treaded_vehicle_first_order_t() {}

    void treaded_vehicle_first_order_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		//euler_integration(simulation_step);
	}

	void treaded_vehicle_first_order_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
	{
		for (int i = 0; i < std::min(lower.size(), lower_bound.size()); ++i)
		{
			lower_bound[i] = lower[i];
		}
		for (int i = 0; i < std::min(upper.size(), upper_bound.size()); ++i)
		{
			upper_bound[i] = upper[i];
		}

		state_space->set_bounds(lower_bound, upper_bound);
	}

	void treaded_vehicle_first_order_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
	}

    void treaded_vehicle_first_order_t::compute_derivative()
    {
		double _yicrL = -.3;
		double _yicrR = .3;

        const double divisor = 1.0/(_yicrL-_yicrR);
		double _vforward = (vr*_yicrL-vl*_yicrR)*divisor;
		double _rot_z = (vl-vr)*divisor;
		if(_vforward*_vforward<PRX_EPSILON*PRX_EPSILON)
		_vforward = 0;
		if(_rot_z*_rot_z<PRX_EPSILON*PRX_EPSILON)
		_rot_z = 0;

		const double c_theta = cos(theta);
		const double s_theta = sin(theta);

		dx = _vforward*c_theta;
		dy = _vforward*s_theta;
		dtheta = _rot_z;
    }
}