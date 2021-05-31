
#include "prx/simulation/plants/racecar_mini.hpp"

namespace prx
{

	racecar_mini_t::racecar_mini_t(const std::string& path) : plant_t(path)
	{

		_x=_y=_psi=_xdot=_ydot=_psidot=0;
		state_memory = {&_x,&_y,&_psi,&_xdot,&_ydot,&_psidot};

		state_space = new space_t("EEREEE",state_memory,"XYTheta");
		state_space->set_bounds(lower_bound,upper_bound);

		_delta = _Fflx = _Ffrx = _Frlx = _Frrx = 0;
		control_memory = {&_delta, &_Fflx, &_Ffrx, &_Frlx, &_Frrx};	
		input_control_space = new space_t("REEEE",control_memory,"V_VTheta");
		input_control_space->set_bounds({-1., -15, -15, -15, -15},{1., 15, 15, 15, 15});

		_xdot = _ydot = _psidot = _xdotdot = _ydotdot = _psidotdot = 0;
		derivative_memory = {&_xdot,&_ydot,&_psidot,&_xdotdot,&_ydotdot,&_psidotdot};
		derivative_space = new space_t("EEEEEE",derivative_memory,"XdotYdotThetadot");

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
		geometries["body"]->initialize_geometry({L, W, H});

		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0x00ff00");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		set_integrator(integrator_t::kEULER);

	}

	void racecar_mini_t::set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
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

	racecar_mini_t::~racecar_mini_t()
	{

	}

	void racecar_mini_t::propagate(const double simulation_step)
	{
		integrator -> integrate(simulation_step);
	}

	void racecar_mini_t::update_configuration()
	{
		auto body = configurations["body"];
		// Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd angle(_psi, Eigen::Vector3d::UnitZ());
		// Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

		// Eigen::Quaternion<double> q = yawAngle;

		body->setIdentity();
		body->translation() = (vector_t(_x, _y, 0));
		body->linear() = angle.toRotationMatrix();
	}

	void racecar_mini_t::compute_derivative()
	{
		using std::isnan;
        using std::isinf;
		using std::max;

		const double C_p = cos(_psi);
		const double S_p = sin(_psi);
		const double C_d = cos(_delta);
		const double S_d = sin(_delta);
		const double C_pd = cos(_psi + _delta);
		const double S_pd = sin(_psi + _delta);

        std::function<double(double,double)> F_ijk = [](double f_ijz, double x)
        {

            return f_ijz * D*sin(C*atan(B * x));
        };

        // returns 1 if x is zero, x otherwise
        std::function<double(double)> is_zero = [](double x){return x==0?1:x;};

        double Fflz = M * g * L2 / ( 2 * L ) - 
                    ( M * H / 2 ) * ( ( _xdotdot / L ) + ( _ydotdot / W) );
        double Ffrz = M * g * L2 / ( 2 * L ) - 
                    ( M * H / 2 ) * ( ( _xdotdot / L ) - ( _ydotdot / W) );
        double Frlz = M * g * L1 / ( 2 * L ) - 
                    ( M * H / 2 ) * ( ( _xdotdot / L ) - ( _ydotdot / W) );
        double Frrz = M * g * L1 / ( 2 * L ) - 
                    ( M * H / 2 ) * ( ( _xdotdot / L ) + ( _ydotdot / W) );

        double lambda_Fr = ( R * _wFr - _xdot ) / is_zero(max(R * _wFr, _xdot)); 
        double lambda_Fl = ( R * _wFl - _xdot ) / is_zero(max(R * _wFl, _xdot)); 
        double lambda_Rr = ( R * _wRr - _xdot ) / is_zero(max(R * _wRr, _xdot)); 
        double lambda_Rl = ( R * _wRl - _xdot ) / is_zero(max(R * _wRl, _xdot)); 

        double sigma = _ydot / is_zero(_xdot);

        double alpha_f = _delta - sigma - L1 * _psidot / is_zero(_xdot);
        double alpha_r = - sigma - L2 * _psidot / is_zero(_xdot);

        double Ffly = F_ijk(Fflz, alpha_f);
        double Ffry = F_ijk(Ffrz, alpha_f);
        double Frly = F_ijk(Frlz, alpha_r);
        double Frry = F_ijk(Frrz, alpha_r);


		_xdotdot = (M * _ydot * _psidot + 
                    C_d * _Fflx + C_d * _Ffrx +
                    S_d * Ffly + S_d * Ffry + 
                    _Frlx + _Frrx) / M;
		_ydotdot = (-M * _xdot * _psidot +
                    S_d * _Fflx + S_d * _Ffrx + 
                    C_d * Ffly + C_d * Ffry + 
                    Frly + Frry) / M;
        _psidotdot = (L1 * ( S_d * _Fflx + S_d * _Ffrx + C_d * Ffly + C_d * Ffry )
                    -L2 * ( Frly + Frry) +
                    W * (-C_d * _Fflx + C_d * _Ffrx + 
                        S_d * Ffly - S_d * Ffry - _Frlx + _Frrx) ) / IZ;
        
        //printf("d's: ( %.3f, %.3f, %.3f )\n", _xdotdot, _ydotdot, _psidotdot);

	}
}
