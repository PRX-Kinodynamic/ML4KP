
#include "prx/simulation/plants/fixed_wing.hpp"

namespace prx
{

	fixed_wing_t::fixed_wing_t(const std::string& path) : plant_t(path)
	{
		// _x=_y=_z=_v=_roll=_pitch=_yaw=_flight=_thrust=0;
		// _V = _alpha = _beta = _p = _q = _r = _phi = _theta_p = _psi = _x = _y = _z = 0;
		_x=_y=_z=_V=_gamma=_chi=_alpha=_mu=_thrust=0;

		state_memory = {&_x,&_y,&_z,
			&_V,&_gamma,&_chi,
			&_alpha,&_mu,&_thrust};
		// state_memory = {&_x  , &_y      , &_z,
		// 				&_V  , &_alpha  , &_beta,
		// 				&_p  , &_q      , &_r,
		// 				&_phi, &_theta_p, &_psi};
		// state_space = new space_t("EEEERRREE",state_memory,"FixedWingState");
		// state_space = new space_t("EEEERRRRRRRR",state_memory,"FixedWingState");
		state_space = new space_t("EEEERRRRE",state_memory,"FixedWingState");

		//						 _x,   _y,   _z, _V,_gamma,  _chi, _alpha,_mu,_thrust
		state_space->set_bounds({-9, -6.5, 1.07,  1, -1.0 , -3.14, -.8, -.8, 0.0},
								{ 9,  6.5, 15  ,  8,  1.0 ,  3.14,  .8,  .8, 8.0});

		// _des_thrust=_des_roll=_des_pitch=0;
		T_c = alpha_c = mu_c = 0;
		// control_memory = {&_des_thrust,&_des_roll,&_des_pitch};
		control_memory = { &T_c,&alpha_c,&mu_c };
		input_control_space = new space_t("ERR",control_memory,"FixedWingControl");
		input_control_space->set_bounds({ 1, -.8, -.8},
										{ 8,  .8,  .8});

		// dx=dy=dz=droll=dpitch=dyaw=dv=dflight=dthrust=0;
		// derivative_memory = {&dx,&dy,&dz,&droll,&dpitch,&dyaw,&dv,&dflight,&dthrust};
		_d_x=_d_y=_d_z=_d_V=_d_gamma=_d_chi=_d_alpha=_d_mu=_d_thrust=0;

		derivative_memory = {&_d_x    , &_d_y      , &_d_z,
							 &_d_V    , &_d_gamma  , &_d_chi,
							 &_d_alpha, &_d_mu     , &_d_thrust};

		// derivative_memory = {&_d_x  , &_d_y      , &_d_z,
		// 				&_d_V  , &_d_alpha  , &_d_beta,
		// 				&_d_p  , &_d_q      , &_d_r,
		// 				&_d_phi, &_d_theta_p, &_d_psi};
		// derivative_space = new space_t("EEEEEEEEE",derivative_memory,"FixedWingDeriv");
		// derivative_space = new space_t("EEEERRRRRRRR",derivative_memory,"FixedWingDeriv");
		derivative_space = new space_t("EEEERRRRE",derivative_memory,"FixedWingDeriv");

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({.25,.1,.07});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0xff0000");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		//set_integrator("rk4");
		set_integrator(integrator_t::kDOPRI5);

	}

	fixed_wing_t::~fixed_wing_t()
	{

	}

	void fixed_wing_t::propagate(const double simulation_step)
	{
		// rk4_integration(simulation_step);
		integrator -> integrate(simulation_step);
	}

	void fixed_wing_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		// const Eigen::AngleAxisd roll_angle(-_roll, Eigen::Vector3d::UnitX());
		// const Eigen::AngleAxisd pitch_angle(-_pitch, Eigen::Vector3d::UnitY());
		// const Eigen::AngleAxisd yaw_angle(_yaw, Eigen::Vector3d::UnitZ());
		// Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
		// body->translation() = (vector_t(_x,_y,_z));
		// body->linear() = q.toRotationMatrix();

		const Eigen::AngleAxisd roll_angle(-_mu, Eigen::Vector3d::UnitX());
		const Eigen::AngleAxisd pitch_angle(-_alpha, Eigen::Vector3d::UnitY());
		const Eigen::AngleAxisd yaw_angle(_chi, Eigen::Vector3d::UnitZ());
		Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
		body->translation() = (vector_t(_x,_y,_z));
		body->linear() = q.toRotationMatrix();
	}

	void fixed_wing_t::compute_derivative()
	{
		// const double G= 9.81;
        // const double k= 1.86;
        const double CL = .3 + 2.5 * _alpha;
        const double CD = .03 + .3 * CL * CL;
        // const double T = T_c / m;

        const double c_alpha = cos(_alpha);
        const double s_alpha = sin(_alpha);

        const double c_chi = cos(_chi);
		const double s_chi = sin(_chi);

		const double c_gamma = cos(_gamma);
		const double s_gamma = sin(_gamma);
        
        const double c_mu = cos(_mu);
        const double s_mu = sin(_mu);

        _d_x = _V * c_gamma * c_chi;
        
        _d_y = _V * c_gamma * s_chi;
        
        _d_z = _V * s_gamma;

        _d_V = (_thrust * c_alpha - k * _V * _V * CD ) - g * s_gamma;

        _d_gamma = ( _thrust * s_alpha / _V + k * _V  * CL) * c_mu - g * c_gamma / _V;

        _d_chi = (_thrust * s_alpha / _V + k * _V * CL) * s_mu / c_gamma;

        _d_thrust = T_c - _thrust;

        _d_alpha = alpha_c - _alpha;

        _d_mu = mu_c - _mu; 






        // const double C_y = 1; // side force
        // const double C_l = 1; // side force
        // const double C_m = 1; // side force
        // const double C_n = -1; // side force
        // const double b = 1; // wing span
        
  //       const double c_beta = cos(_beta);
  //       const double s_beta = sin(_beta);
  //       const double t_beta = tan(_beta);
  //       const double c_theta_p = cos(_theta_p);
  //       const double s_theta_p = sin(_theta_p);
  //       const double t_theta_p = tan(_theta_p);
  //       const double sec_theta_p = 1.0 / c_theta_p;
  //       const double c_phi = cos(_phi);
  //       const double s_phi = sin(_phi);
  //       const double c_psi = cos(_psi);
  //       const double s_psi = sin(_psi);

  //       const double sin_gamma = c_alpha * c_beta * s_theta_p - s_beta * s_phi * c_theta_p - s_alpha * c_beta * c_theta_p * c_phi;
  //       // gamma = asin(sin_gamma);
  //       const double c_mu_c_gamma = s_alpha * s_theta_p + c_alpha * c_theta_p * c_phi;

  //       const double s_mu_c_gamma = c_alpha * s_beta * s_theta_p + c_beta * s_phi * c_theta_p - s_alpha * s_beta * c_phi * c_theta_p;

  //       const double c_gamma = cos(asin(sin_gamma));
  //       const double s_chi_c_gamma = c_alpha * c_beta * c_theta_p * s_psi + s_beta * ( s_phi * s_theta_p * s_psi + c_phi * c_psi ) + 
  //       	s_alpha * c_beta * ( c_phi * s_theta_p * s_psi - s_phi * c_psi ); 

  //       const double chi = asin(s_chi_c_gamma / c_gamma);

  //       _d_V = (1.0 / m ) * ( T - 0.5 * rho * _V * S * CD - m * g * sin_gamma);

  //       _d_alpha = _q - t_beta * (_p * c_alpha + _r * s_alpha) - ( 1.0 / (m * _V * c_beta) 
  //       	* ( T * s_alpha / _V + 0.5 * rho * _V * _V * S * CL - m * g * c_mu * c_mu_c_gamma ) );

  //       _d_beta = _p * s_alpha - _r * c_alpha + ( 1.0 / ( m * _V ) ) * 
  //       	( m * g * s_mu_c_gamma + 0.5 * rho * _V * _V * S * C_y  - T * c_alpha * c_beta) ;

  //       _d_p = ( ( I_y - I_z ) / I_x ) * _q * _r + (1.0 / I_x) * ( 0.5 * rho * _V * _V * S * wing_span * C_l );
        
  //       _d_q = ( ( I_z - I_x ) / I_y ) * _r * _p + (1.0 / I_y) * ( 0.5 * rho * _V * _V * S * wing_chord * C_m );

  //       _d_r = ( ( I_x - I_y ) / I_z ) * _p * _q + (1.0 / I_z) * ( 0.5 * rho * _V * _V * S * wing_span * C_n );

  //       _d_phi = _p + t_theta_p * ( _q * s_phi + _r * c_phi );

  //       _d_theta_p = _q * c_phi - _r * s_phi;

  //       _d_psi = sec_theta_p * ( _r * c_phi + _q * s_phi );

  //       _d_x = _V * c_gamma * c_chi;
  //       _d_y = _V * c_gamma * s_chi;
  //       _d_z = _V * sin_gamma ;

        // 9 state vars OLD
        // const double cflight = cos(_flight);
        // const double sflight = sin(_flight);
        // const double spitch = sin(_pitch);
        // dx =       _v * cflight * cos(_yaw);
        // dy =       _v * cflight * sin(_yaw);
        // dz =       _v * sflight ;
        // dv =       ( _thrust*cos(_pitch) - k*_v*_v*CD ) - G*sflight;
        // dflight =  ( _thrust*spitch/_v + k*_v*CL )*cos(_roll) - G*cflight/_v;
        // dyaw =     ( _thrust*spitch/_v + k*_v*CL )*(sin(_roll)/cflight);
        // dthrust =  _des_thrust-_thrust;
        // droll =    _des_roll-_roll;
        // dpitch =   _des_pitch-_pitch;
	}
}


