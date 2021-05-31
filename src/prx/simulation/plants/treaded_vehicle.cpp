
#include "prx/simulation/plants/treaded_vehicle.hpp"

namespace prx
{

	treaded_vehicle_t::treaded_vehicle_t(const std::string& path) : plant_t(path)
	{
		x=y=theta=vl=vr=0;
		state_memory = {&x,&y,&theta,&vl,&vr};
		state_space = new space_t("EEREE",state_memory,"TreadedState");
		//state_space->set_bounds({-100,-100,-3.15,-.1,-.1},{100,100,3.15,.7,.7});
		state_space->set_bounds(lower_bound,upper_bound);

		al=ar=0;
		control_memory = {&al,&ar};
		input_control_space = new space_t("EE",control_memory,"TreadedControl");
		input_control_space->set_bounds({-.2,-.2},{.2,.2});

		dx=dy=dtheta=0;
		derivative_memory = {&dx,&dy,&dtheta,&al,&ar};
		derivative_space = new space_t("EEEEE",derivative_memory,"ThetaDerivative");

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.9,.6,.25});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0xff00ff");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();
		use_cost_map = false;

		set_integrator(integrator_t::kEULER);

	}

	treaded_vehicle_t::~treaded_vehicle_t()
	{

	}

	void treaded_vehicle_t::compute_stopping_maneuver(space_point_t start_state, std::vector<double>& times, std::vector<double>& ctrls)//, std::vector<double> * controls)
	{
		double stop_l = fabs(start_state->at(3) / 0.2);
		double stop_r = fabs(start_state->at(4) / 0.2);
		times.push_back(stop_l);
		times.push_back(stop_r);
		if (start_state->at(3) < 0)
			ctrls.push_back(0.2);
		else
			ctrls.push_back(-0.2);
		if (start_state->at(4) < 0)
			ctrls.push_back(0.2);
		else
			ctrls.push_back(-0.2);
		// std::cout << "time Left: " << stop_l << "\tRight: " << stop_r << std::endl;
		// std::cout << "control Left: " << (*ctrls)[0] << "\tRight: " << (*ctrls)[1] << std::endl;
		// std::cout << "Acc Left: " << al << "\tRight: " << ar << std::endl;
	}

	void treaded_vehicle_t::set_cost_map(double ** c_map, double d_x, double d_y, int g_h, int g_w, double c_size)
	{
		if (!use_cost_map)
		{
			cost_map = c_map;
			delta_x = d_x;
			delta_y = d_y;
			cell_size = c_size;
			grid_height = g_h;
			grid_width = g_w;
			auto to_global_space = [&d_x, &d_y, &c_size](int coord, bool is_x_val)
			{
				double d_xy = is_x_val ? d_x : d_y;
				return (coord + d_xy )* c_size;
			};
			state_space->set_bounds({to_global_space(0, true),to_global_space(0, false),-3.15,-.1,-.1},
							{to_global_space(grid_height, true),to_global_space(grid_width, false), 3.15, .7, .7});
			use_cost_map = true;
		}

	}

	void treaded_vehicle_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
		//euler_integration(simulation_step);
	}

	void treaded_vehicle_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x,y,0));
	}

	void treaded_vehicle_t::compute_derivative()
	{
		double _xicr = 0;
		double _yicrL = -.3;
		double _yicrR = .3;

		double traversability = 1.0;
		if(use_cost_map)
		{
			double d_x = delta_x, d_y = delta_y, c_size = cell_size;
			int g_h = grid_height, g_w = grid_width;
			auto to_map_space = [&d_x, &d_y, &g_h, &g_w, &c_size](double coord, bool is_x_val)
			{
				double d_xy = is_x_val ? d_x : d_y;
				int max_coord = is_x_val ? g_h-1 : g_w-1;
				int out = coord / c_size - d_xy;
				out = std::min(out, max_coord);
				out = std::max(out, 0);
				return out;
			};
			int x_val = to_map_space(x, true);
			int y_val = to_map_space(y, false);
			//printf("%d %d %f %f %f %f\n", x_val, y_val, x, delta_x, y, delta_y);
		      traversability = 1-cost_map[x_val][y_val];
		}
		_xicr = (traversability>.5?0:.6*(.5-traversability)*2);
		_yicrL = -((traversability)*.3+(1.0-traversability)*2.0);
		_yicrR = ((traversability)*.3+(1.0-traversability)*2.0);

		const double divisor = 1.0/(_yicrL-_yicrR);
		double _vforward = (vr*_yicrL-vl*_yicrR)*divisor;
		double _vlateral = -(vr*_xicr-vl*_xicr)*divisor;
		double _rot_z = (vl-vr)*divisor;
		if(_vforward*_vforward<PRX_EPSILON*PRX_EPSILON)
		_vforward = 0;
		if(_vlateral*_vlateral<PRX_EPSILON*PRX_EPSILON)
		_vlateral = 0;
		if(_rot_z*_rot_z<PRX_EPSILON*PRX_EPSILON)
		_rot_z = 0;

		const double c_theta = cos(theta);
		const double s_theta = sin(theta);

		dx = _vforward*c_theta-_vlateral*s_theta;
		dy = _vforward*s_theta+_vlateral*c_theta;
		dtheta = _rot_z;
	}
}
