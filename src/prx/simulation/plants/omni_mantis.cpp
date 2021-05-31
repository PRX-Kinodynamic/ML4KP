
#include "prx/simulation/plants/omni_mantis.hpp"

namespace prx
{


	quaternion_t set_from_euler( double roll, double pitch, double yaw )
	{
		roll*=.5;
		pitch*=.5;
		yaw*=.5;
		double sinroll = sin(roll);
		double sinpitch = sin(pitch);
		double sinyaw = sin(yaw);
		double cosroll = cos(roll);
		double cospitch = cos(pitch);
		double cosyaw = cos(yaw);
		quaternion_t quat;

		quat.x() = sinroll*cospitch*cosyaw - cosroll*sinpitch*sinyaw;
		quat.y() = cosroll*sinpitch*cosyaw + sinroll*cospitch*sinyaw;
		quat.z() = cosroll*cospitch*sinyaw - sinroll*sinpitch*cosyaw;
		quat.w() = cosroll*cospitch*cosyaw + sinroll*sinpitch*sinyaw;
		return quat;
	}

	omni_mantis_t::omni_mantis_t(const std::string& path) : plant_t(path)
	{

		state_memory =      {&_x, &_y, &_z, &_theta, &_vx,&_vy,&_vz,&_vtheta};
		control_memory = {&_ax,&_ay,&_az,&_atheta};
		derivative_memory = {&_dx,&_dy,&_vz,&_vtheta,&_ax,&_ay,&_az,&_atheta};

		state_space = new space_t("EEEREEEE",state_memory,"OmniState");
		state_space->set_bounds({-30,-30,  .5,-3.14,   0,-.75,-.5,-.25},
								{ 30, 30, 1.7, 3.14, 1.5, .75, .5, .25});

		input_control_space = new space_t("EEEE",control_memory,"OmniControl");
		input_control_space->set_bounds({ -.5, -.5, -.5, -.1},
										{  .5,  .5,  .5,  .1});

		derivative_space = new space_t("EEEEEEEE",derivative_memory,"OmniDerivative");

		geometries["chassis"] = std::make_shared<geometry_t>(geometry_type_t::ELLIPSOID);
		geometries["chassis"]->initialize_geometry({.75,.5,.2});
		geometries["chassis"]->generate_collision_geometry();
		geometries["chassis"]->set_visualization_color("0xc0c0c0");
		configurations["chassis"]= std::make_shared<transform_t>();
		configurations["chassis"]->setIdentity();

		geometries["lf_shoulder"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["lf_shoulder"]->initialize_geometry({.15,1.5});
		geometries["lf_shoulder"]->generate_collision_geometry();
		geometries["lf_shoulder"]->set_visualization_color("0xff0000");
		configurations["lf_shoulder"]= std::make_shared<transform_t>();
		configurations["lf_shoulder"]->setIdentity();

		geometries["rf_shoulder"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["rf_shoulder"]->initialize_geometry({.15,1.5});
		geometries["rf_shoulder"]->generate_collision_geometry();
		geometries["rf_shoulder"]->set_visualization_color("0xff0000");
		configurations["rf_shoulder"]= std::make_shared<transform_t>();
		configurations["rf_shoulder"]->setIdentity();

		geometries["lr_shoulder"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["lr_shoulder"]->initialize_geometry({.15,1.5});
		geometries["lr_shoulder"]->generate_collision_geometry();
		geometries["lr_shoulder"]->set_visualization_color("0xff0000");
		configurations["lr_shoulder"]= std::make_shared<transform_t>();
		configurations["lr_shoulder"]->setIdentity();

		geometries["rr_shoulder"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["rr_shoulder"]->initialize_geometry({.15,1.5});
		geometries["rr_shoulder"]->generate_collision_geometry();
		geometries["rr_shoulder"]->set_visualization_color("0xff0000");
		configurations["rr_shoulder"]= std::make_shared<transform_t>();
		configurations["rr_shoulder"]->setIdentity();

		geometries["lf_arm"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["lf_arm"]->initialize_geometry({.15,.5});
		geometries["lf_arm"]->generate_collision_geometry();
		geometries["lf_arm"]->set_visualization_color("0xff0000");
		configurations["lf_arm"]= std::make_shared<transform_t>();
		configurations["lf_arm"]->setIdentity();

		geometries["rf_arm"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["rf_arm"]->initialize_geometry({.15,.5});
		geometries["rf_arm"]->generate_collision_geometry();
		geometries["rf_arm"]->set_visualization_color("0xff0000");
		configurations["rf_arm"]= std::make_shared<transform_t>();
		configurations["rf_arm"]->setIdentity();

		geometries["lr_arm"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["lr_arm"]->initialize_geometry({.15,.5});
		geometries["lr_arm"]->generate_collision_geometry();
		geometries["lr_arm"]->set_visualization_color("0xff0000");
		configurations["lr_arm"]= std::make_shared<transform_t>();
		configurations["lr_arm"]->setIdentity();

		geometries["rr_arm"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
		geometries["rr_arm"]->initialize_geometry({.15,.5});
		geometries["rr_arm"]->generate_collision_geometry();
		geometries["rr_arm"]->set_visualization_color("0xff0000");
		configurations["rr_arm"]= std::make_shared<transform_t>();
		configurations["rr_arm"]->setIdentity();

		geometries["lf_wheel"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["lf_wheel"]->initialize_geometry({.3,.15});
		geometries["lf_wheel"]->generate_collision_geometry();
		geometries["lf_wheel"]->set_visualization_color("0x000000");
		configurations["lf_wheel"]= std::make_shared<transform_t>();
		configurations["lf_wheel"]->setIdentity();

		geometries["rf_wheel"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["rf_wheel"]->initialize_geometry({.3,.15});
		geometries["rf_wheel"]->generate_collision_geometry();
		geometries["rf_wheel"]->set_visualization_color("0x000000");
		configurations["rf_wheel"]= std::make_shared<transform_t>();
		configurations["rf_wheel"]->setIdentity();

		geometries["lr_wheel"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["lr_wheel"]->initialize_geometry({.3,.15});
		geometries["lr_wheel"]->generate_collision_geometry();
		geometries["lr_wheel"]->set_visualization_color("0x000000");
		configurations["lr_wheel"]= std::make_shared<transform_t>();
		configurations["lr_wheel"]->setIdentity();

		geometries["rr_wheel"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
		geometries["rr_wheel"]->initialize_geometry({.3,.15});
		geometries["rr_wheel"]->generate_collision_geometry();
		geometries["rr_wheel"]->set_visualization_color("0x000000");
		configurations["rr_wheel"]= std::make_shared<transform_t>();
		configurations["rr_wheel"]->setIdentity();

		transforms.push_back(configurations["chassis"]);
		transforms.push_back(configurations["lf_shoulder"]);
		transforms.push_back(configurations["rf_shoulder"]);
		transforms.push_back(configurations["lr_shoulder"]);
		transforms.push_back(configurations["rr_shoulder"]);
		transforms.push_back(configurations["lf_arm"]);
		transforms.push_back(configurations["rf_arm"]);
		transforms.push_back(configurations["lr_arm"]);
		transforms.push_back(configurations["rr_arm"]);
		transforms.push_back(configurations["lf_wheel"]);
		transforms.push_back(configurations["rf_wheel"]);
		transforms.push_back(configurations["lr_wheel"]);
		transforms.push_back(configurations["rr_wheel"]);

		set_integrator(integrator_t::kRK4);

	}

	omni_mantis_t::~omni_mantis_t()
	{

	}

	void omni_mantis_t::propagate(const double simulation_step)
	{
		// rk4_integration(simulation_step);
		integrator -> integrate(simulation_step);
		

	}

	void omni_mantis_t::update_configuration()
	{
		const double s_theta = sin(_theta);
		const double c_theta = cos(_theta);
		const double sz = _z + .25;
		const double az = .5 + .25;
		transforms[0]->setIdentity();
		transforms[1]->setIdentity();
		transforms[2]->setIdentity();
		transforms[3]->setIdentity();
		transforms[4]->setIdentity();
		transforms[5]->setIdentity();
		transforms[6]->setIdentity();
		transforms[7]->setIdentity();
		transforms[8]->setIdentity();
		transforms[9]->setIdentity();
		transforms[10]->setIdentity();
		transforms[11]->setIdentity();
		transforms[12]->setIdentity();

		transforms[0]->translation() = (vector_t(_x,_y,sz));
		transforms[0]->linear() = (quaternion_t(cos(_theta/2),0,0,sin(_theta/2)).toRotationMatrix());
	
		//distance from shoulder joint to arm joint in the x-y plane
		const double arm_magnitude = sqrt(1.5*1.5-(sz-az)*(sz-az));
		const double rx = .75;
		const double ry = .5;
		const double shoulder_angle_theta = tan(PRX_PI/6.0);
		const double shoulder_x = rx*ry/(sqrt(ry*ry+rx*rx*shoulder_angle_theta*shoulder_angle_theta));
		const double shoulder_y = rx*ry/(sqrt(rx*rx+ry*ry/(shoulder_angle_theta*shoulder_angle_theta)));
		const double shoulder_mag = sqrt(shoulder_x*shoulder_x+shoulder_y*shoulder_y);

		const auto start_vec = vector_t(0,0,1);

		double wheel_direction = PRX_PI + _theta;
		if( fabs(_vx*s_theta+_vy*c_theta)>PRX_EPSILON || 
			fabs(_vx*c_theta-_vy*s_theta)>PRX_EPSILON)
		{
			wheel_direction = PRX_PI + atan2(_vx*s_theta+_vy*c_theta,_vx*c_theta-_vy*s_theta);
		}

		const quaternion_t quat = set_from_euler(PRX_PI/2,0,wheel_direction);
		auto matrix = quat.toRotationMatrix();

		const auto joint_values = [&,this](double sx, double sy, int start_index)
		{
			//arm joint position
			const double ax = (sx/shoulder_mag)*(shoulder_mag+arm_magnitude);
			const double ay = (sy/shoulder_mag)*(shoulder_mag+arm_magnitude);

			const double rot_sx = sx*c_theta-sy*s_theta;
			const double rot_sy = sx*s_theta+sy*c_theta;
			const double rot_ax = ax*c_theta-ay*s_theta;
			const double rot_ay = ax*s_theta+ay*c_theta;

			const auto end_vec = vector_t(rot_sx - rot_ax,rot_sy - rot_ay,sz - az);

			transforms[start_index]->translation() = (vector_t(_x+(rot_sx+rot_ax)/2,_y+(rot_sy+rot_ay)/2,(sz+az)/2));
			transforms[start_index]->linear() = quaternion_t().setFromTwoVectors(start_vec,end_vec).toRotationMatrix();
			transforms[start_index+4]->translation() = (vector_t(_x+rot_ax,_y+rot_ay,az-.25));
			transforms[start_index+4]->linear() = quaternion_t(1,0,0,0).toRotationMatrix();
			transforms[start_index+8]->translation() = (vector_t(_x+rot_ax,_y+rot_ay,az-.5));
			transforms[start_index+8]->linear() = matrix;
		};

		joint_values(-shoulder_x, shoulder_y,1);
		joint_values( shoulder_x, shoulder_y,2);
		joint_values(-shoulder_x,-shoulder_y,3);
		joint_values( shoulder_x,-shoulder_y,4);

	}

	void omni_mantis_t::compute_derivative()
	{
		_dx = _vx*cos(_theta)-_vy*sin(_theta);
		_dy = _vx*sin(_theta)+_vy*cos(_theta);
	}
}