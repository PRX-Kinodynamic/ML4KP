
#include "prx/simulation/plants/koules.hpp"

namespace prx
{

	koules_t::koules_t(const std::string& path, int num_koules) : plant_t(path)
	{
		koules_size = num_koules;
		koules_state.resize(koules_size,std::vector<double>(4));
		koules_deriv.resize(koules_size,std::vector<double>(2));

		x=y=theta=vx=vy=0;
		state_memory = {&x,&y,&theta,&vx,&vy};

		std::string state_topo = "EEREE";

		std::vector<double> min_state_bounds = {0, 0,-3.15,-1,-1};
		std::vector<double> max_state_bounds = {1, 1, 3.15, 1, 1};


		for(int i=0;i<koules_size;i++)
		{
			state_memory.push_back(&koules_state[i][0]);
			state_memory.push_back(&koules_state[i][1]);
			state_memory.push_back(&koules_state[i][2]);
			state_memory.push_back(&koules_state[i][3]);
			state_topo += "EEEE";

			min_state_bounds.push_back(-1);
			min_state_bounds.push_back(-1);
			min_state_bounds.push_back(-3);
			min_state_bounds.push_back(-3);
			max_state_bounds.push_back(1);
			max_state_bounds.push_back(1);
			max_state_bounds.push_back(3);
			max_state_bounds.push_back(3);
		}

		state_space = new space_t(state_topo,state_memory,"Koules"+std::to_string(koules_size));
		state_space->set_bounds(min_state_bounds,max_state_bounds);

		vtheta=a=0;
		control_memory = {&vtheta,&a};
		input_control_space = new space_t("EE",control_memory,"KoulesControl");
		input_control_space->set_bounds({-PRX_PI,0},{PRX_PI,1});


		std::string deriv_topo = "EEEEE";

		ax=ay=0;
		derivative_memory = {&vx,&vy,&vtheta,&ax,&ay};

		for(int i=0;i<koules_size;i++)
		{
			derivative_memory.push_back(&koules_state[i][2]);
			derivative_memory.push_back(&koules_state[i][3]);
			derivative_memory.push_back(&koules_deriv[i][0]);
			derivative_memory.push_back(&koules_deriv[i][1]);
			deriv_topo += "EEEE";
		}

		robot_radius = .03;
		koule_radius = .015;

		derivative_space = new space_t(deriv_topo,derivative_memory,"KoulesDeriv"+std::to_string(koules_size));

		geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
		geometries["body"]->initialize_geometry({robot_radius*100});
		geometries["body"]->generate_collision_geometry();
		geometries["body"]->set_visualization_color("0x00ff00");
		configurations["body"]= std::make_shared<transform_t>();
		configurations["body"]->setIdentity();

		for(int i=0;i<koules_size;i++)
		{
			geometries["koule"+std::to_string(i)] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
			geometries["koule"+std::to_string(i)]->initialize_geometry({koule_radius*100});
			geometries["koule"+std::to_string(i)]->generate_collision_geometry();
			if(i==0)
				geometries["koule"+std::to_string(i)]->set_visualization_color("0xff0000");
			if(i==1)
				geometries["koule"+std::to_string(i)]->set_visualization_color("0xffff00");
			if(i==2)
				geometries["koule"+std::to_string(i)]->set_visualization_color("0xff00ff");
			if(i==3)
				geometries["koule"+std::to_string(i)]->set_visualization_color("0x00ffff");
			configurations["koule"+std::to_string(i)]= std::make_shared<transform_t>();
			configurations["koule"+std::to_string(i)]->setIdentity();
		}

		prev_state = state_space->make_point();
		new_state = state_space->make_point();

		set_integrator(integrator_t::kEULER);

	}

	koules_t::~koules_t()
	{

	}

	n_vector_t<2> collision_response(n_vector_t<2> x1, n_vector_t<2> x2, 
									 n_vector_t<2> v1, n_vector_t<2> v2,
									 double m1, double m2)
	{
		const double mass_scale = 2*m2/(m1+m2);
		const auto v1v2 = v1-v2;
		const auto x1x2 = x1-x2;
		const double sq_norm = x1x2.squaredNorm();

		return v1 - (mass_scale/sq_norm)*v1v2.dot(x1x2)*x1x2;
	}

	void koules_t::propagate(const double simulation_step)
	{
		state_space->copy_to_point(prev_state);
		integrator -> integrate(simulation_step);
		//euler_integration(simulation_step);
		state_space->copy_to_point(new_state);
		//post process collisions
		const double mass_s = .75;
		const double mass_k = .5;

		//check koule boundary collisions
		int successes = 0;
		for(int i=0;i<koules_size;i++)
		{
			if(koules_state[i][0]<koule_radius || koules_state[i][0]>1.0-koule_radius
			  || koules_state[i][1]<koule_radius || koules_state[i][1]>1.0-koule_radius)
			{
				koules_state[i][0]=-1;
				koules_state[i][1]=-1;
				koules_state[i][2]=0;
				koules_state[i][3]=0;
				new_state->at(5+i*4+0) = -1;
				new_state->at(5+i*4+1) = -1;
				new_state->at(5+i*4+2) = 0;
				new_state->at(5+i*4+3) = 0;
				successes++;
			}
		}

		//check koule, koule collisions
		for(int i=0;i<koules_size;i++)
		{
			if(koules_state[i][0]==-1)
			{
				continue;
			}
			for(int j=i+1;j<koules_size;j++)
			{
				//check if we collide
				double rad2 = (koule_radius+koule_radius)*(koule_radius+koule_radius);
				double dist2 = (koules_state[i][0]-koules_state[j][0])*(koules_state[i][0]-koules_state[j][0]) +
					 		   (koules_state[i][1]-koules_state[j][1])*(koules_state[i][1]-koules_state[j][1]);

				if(dist2 <= rad2)
				{
					//collision occurred, update velocity terms
					n_vector_t<2> x1(prev_state->at(5+i*4+0),prev_state->at(5+i*4+1));
					n_vector_t<2> x2(prev_state->at(5+j*4+0),prev_state->at(5+j*4+1));

					n_vector_t<2> v1(koules_state[i][2],koules_state[i][3]);
					n_vector_t<2> v2(koules_state[j][2],koules_state[j][3]);

					auto v1_new = collision_response(x1,x2,v1,v2,mass_k,mass_k);
					auto v2_new = collision_response(x2,x1,v2,v1,mass_k,mass_k);

					new_state->at(5+i*4+2) = v1_new(0);
					new_state->at(5+i*4+3) = v1_new(1);
					new_state->at(5+j*4+2) = v2_new(0);
					new_state->at(5+j*4+3) = v2_new(1);

					new_state->at(5+i*4+0) = prev_state->at(5+i*4+0);
					new_state->at(5+i*4+1) = prev_state->at(5+i*4+1);
					new_state->at(5+j*4+0) = prev_state->at(5+j*4+0);
					new_state->at(5+j*4+1) = prev_state->at(5+j*4+1);
				}
			}
		}
		for(int i=0;i<koules_size;i++)
		{
			if(koules_state[i][0]==-1)
			{
				continue;
			}
			//check if we collide
			double rad2 = (koule_radius+robot_radius)*(koule_radius+robot_radius);
			double dist2 = (koules_state[i][0]-x)*(koules_state[i][0]-x) +
				 		   (koules_state[i][1]-y)*(koules_state[i][1]-y);

			if(dist2 <= rad2)
			{
				//collision occurred, update velocity terms

				n_vector_t<2> x1(prev_state->at(0),prev_state->at(1));
				n_vector_t<2> x2(prev_state->at(5+i*4+0),prev_state->at(5+i*4+1));

				n_vector_t<2> v1(vx,vy);
				n_vector_t<2> v2(koules_state[i][2],koules_state[i][3]);

				auto v1_new = collision_response(x1,x2,v1,v2,mass_k,mass_k);
				auto v2_new = collision_response(x2,x1,v2,v1,mass_k,mass_k);

				new_state->at(5+i*4+2) = v2_new(0);
				new_state->at(5+i*4+3) = v2_new(1);
				new_state->at(2) = v1_new(0);
				new_state->at(3) = v1_new(1);
				new_state->at(5+i*4+0) = prev_state->at(5+i*4+0);
				new_state->at(5+i*4+1) = prev_state->at(5+i*4+1);
				new_state->at(0) = prev_state->at(0);
				new_state->at(1) = prev_state->at(1);
			}
		}
		state_space->copy_from_point(new_state);

	}

	void koules_t::update_configuration()
	{
		auto body = configurations["body"];
		body->setIdentity();
		body->linear() = (quaternion_t(cos(theta/2),0,0,sin(theta/2)).toRotationMatrix());
		body->translation() = (vector_t(x*100-50,y*100-50,0));

		for(int i=0;i<koules_size;i++)
		{
			auto body = configurations["koule"+std::to_string(i)];
			body->setIdentity();
			body->translation() = (vector_t(koules_state[i][0]*100-50,koules_state[i][1]*100-50,0));
		}
	}

	void koules_t::compute_derivative()
	{
		ax = a*cos(theta);
		ay = a*sin(theta);

		const double h = .05;
		const double lambda_c = 4;

		for(int i=0;i<koules_size;i++)
		{
			if(koules_state[i][0]!=-1 && koules_state[i][1]!=-1)
			{
				koules_deriv[i][0] = (.5 - koules_state[i][0])*lambda_c - koules_state[i][2]*h;
				koules_deriv[i][1] = (.5 - koules_state[i][1])*lambda_c - koules_state[i][3]*h;
			}
		}

	}
}