
#include "prx/planning/planner_functions/configurable_functions.hpp"

namespace prx
{
	configurable_functions_t::configurable_functions_t(std::string context_name,  world_model_context context, param_loader pl)
	{
		// auto context = wm->get_context(context_name);
		auto sg = context.first;
		auto cg = context.second;
		auto state_space = sg->get_state_space();
		auto control_space = sg->get_control_space();

		int min_steps = pl["control_min"].as<int>();
		int max_steps = pl["control_max"].as<int>();
		auto plant_type = pl["plant_type"].as<std::string>();

		if(plant_type=="koules")
		{
			int num_koules = (state_space->get_dimension()-5)/4;
			auto bounds = state_space->get_bounds();
			sample_state = [bounds,num_koules](space_point_t& s)
			{
				for(int i=0;i<5;i++)
				{
					s->at(i) = uniform_random(bounds[i].first,bounds[i].second);
				}
				for(int k=0;k<num_koules;k++)
				{
					s->at(5+4*k) = uniform_random(uniform_random()<.5?-1:0,bounds[5+4*k].second);
					s->at(5+4*k+1) = uniform_random(uniform_random()<.5?-1:0,bounds[5+4*k+1].second);
					s->at(5+4*k+2) = uniform_random(bounds[5+4*k+2].first,bounds[5+4*k+2].second);
					s->at(5+4*k+3) = uniform_random(bounds[5+4*k+3].first,bounds[5+4*k+3].second);
				}
			};
			valid_state = [](space_point_t& s)
			{
				if(s->at(0)<.03 || s->at(0)>1.0-.03
				  || s->at(1)<.03 || s->at(1)>1.0-.03)
				{
					return false;
				}
				return true;
			};
		}
		else
		{
			sample_state = [state_space](space_point_t& s)
			{
				default_sample_state(s,state_space);
			};
			valid_state = [state_space,cg](space_point_t& s)
			{
				return default_valid_state(s, state_space,cg);
			};
		}

		/////////////////////////////////////
		sample_plan = [control_space,min_steps,max_steps](plan_t& p, space_point_t pt)
		{
			default_sample_plan(p,control_space,min_steps,max_steps);
		};
		valid_check = [this](trajectory_t& traj)
		{
			for(auto s : traj)
			{
				if(!valid_state(s))
					return false;
			}
			return true;
		};
		propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj)
		{
			default_propagate(start_state,plan,out_traj,sg);
		};
		expand = [sg,this](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
		{
			default_expand(s, plans, trajs, bn, sg, sample_plan, propagate);
		};
		/////////////////////////////////////


		if(plant_type=="2d_point")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
							(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)));
			};
		}
		else if(plant_type=="3d_point")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
							(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1))+
							(s1->at(2)-s2->at(2))*(s1->at(2)-s2->at(2)));
			};
		}
		else if(plant_type=="two_link_acrobot")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
	            double ddot = fabs(s1->at(2) - s2->at(2));
	            double ddot2 = fabs(s1->at(3) - s2->at(3));
	            return pow((sin(s2->at(0)) + sin(s2->at(0) + s2->at(1)) - (sin(s1->at(0)) + sin(s1->at(0) + s1->at(1)))),2.0)
	                    + pow((cos(s2->at(0)) + cos(s2->at(0) + s2->at(1)) - (cos(s1->at(0)) + cos(s1->at(0) + s1->at(1)))),2.0);
			};
		}
		else if(plant_type=="fixed_wing")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
							(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1))+
							(s1->at(2)-s2->at(2))*(s1->at(2)-s2->at(2)));
				// return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
				// 			(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1))+
				// 			(s1->at(2)-s2->at(2))*(s1->at(2)-s2->at(2))+
				// 			(s1->at(3)-s2->at(3))*(s1->at(3)-s2->at(3)));
			};
		}
		else if(plant_type=="mantis")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
							(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1))+
							(s1->at(2)-s2->at(2))*(s1->at(2)-s2->at(2)));
			};
		}
		else if(plant_type=="treaded_vehicle" || plant_type=="treaded_vehicle_fo" || plant_type == "racecar")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
							(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)));
			};
		}
		else if(plant_type=="IAI_plant")
		{
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{			   
			  return sqrt((s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
				      (s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1)))/1.0;
			};
		}
		else if(plant_type=="koules")
		{
			int num_koules = (state_space->get_dimension()-5)/4;
			distance_function = [num_koules](const space_point_t& s1, const space_point_t& s2)
			{
				double value = (s1->at(0)-s2->at(0))*(s1->at(0)-s2->at(0))+
							(s1->at(1)-s2->at(1))*(s1->at(1)-s2->at(1))+
							(s1->at(3)-s2->at(3))*(s1->at(3)-s2->at(3))+
							(s1->at(4)-s2->at(4))*(s1->at(4)-s2->at(4));

				double angle_diff = s1->at(2) - s2->at(2);
				angle_diff = fabs(norm_angle_pi(angle_diff + PRX_PI) - PRX_PI);
				value+=angle_diff;

				for(int k=0;k<num_koules;k++)
				{
					value+=(s1->at(5+4*k+0)-s2->at(5+4*k+0))*(s1->at(5+4*k+0)-s2->at(5+4*k+0))+
							(s1->at(5+4*k+1)-s2->at(5+4*k+1))*(s1->at(5+4*k+1)-s2->at(5+4*k+1))+
							(s1->at(5+4*k+2)-s2->at(5+4*k+2))*(s1->at(5+4*k+2)-s2->at(5+4*k+2))+
							(s1->at(5+4*k+3)-s2->at(5+4*k+3))*(s1->at(5+4*k+3)-s2->at(5+4*k+3));
				}
				return sqrt(value);
			};
		}
		else
		{
			prx_throw("Unable to find a distance function in configurable_functions_t for the specified plant "<<plant_type);
		}

		auto cost_function_type = pl["cost_function_type"].as<std::string>();

		if(cost_function_type=="distance")
		{
			cost_function = [this](const trajectory_t& t, const plan_t& plan)
			{
				double dist = 0;
				double step = .1;
				for(double i=0;i<=1.0-step;i+=step)
				{
					dist+=distance_function(t[i],t[i+step]);
				}
				return dist;
			};

			h = [this](const space_point_t& s, const space_point_t& s2)
			{
				return distance_function(s,s2);
			};

		}
		else if(cost_function_type=="time")
		{
			cost_function = [](const trajectory_t& t, const plan_t& plan)
			{
				return plan.duration();
			};

			double velocity_factor = 1.0;
			auto s_bounds = state_space->get_bounds();
			auto c_bounds = control_space->get_bounds();
			if(plant_type=="fixed_wing")
			{
				velocity_factor = s_bounds[3].second;
			}
			else if(plant_type=="koules")
			{
				velocity_factor = sqrt(	s_bounds[3].second*s_bounds[3].second
										+s_bounds[4].second*s_bounds[4].second);
			}
			else if(plant_type=="mantis")
			{
				velocity_factor = sqrt(	s_bounds[4].second*s_bounds[4].second
										+s_bounds[5].second*s_bounds[5].second
										+s_bounds[6].second*s_bounds[6].second);
			}
			else if(plant_type=="3d_point")
			{
				velocity_factor = sqrt(	c_bounds[0].second*c_bounds[0].second
										+c_bounds[1].second*c_bounds[1].second
										+c_bounds[2].second*c_bounds[2].second);
			}
			else if(plant_type=="treaded_vehicle")
			{
				velocity_factor = s_bounds[3].second;
			}
			else if(plant_type=="2d_point")
			{
				velocity_factor = sqrt(	c_bounds[0].second*c_bounds[0].second
										+c_bounds[1].second*c_bounds[1].second);
			}
			else if(plant_type=="two_link_acrobot")
			{
				velocity_factor = s_bounds[1].second;
			}

			if (plant_type=="koules")
			{

				int num_koules = (state_space->get_dimension()-5)/4;
				h = [this,velocity_factor,num_koules](const space_point_t& s, const space_point_t& s2)
				{
					double value = (s->at(0)-s2->at(0))*(s->at(0)-s2->at(0))+
								(s->at(1)-s2->at(1))*(s->at(1)-s2->at(1));

					for(int k=0;k<num_koules;k++)
					{
						value+=(s->at(5+4*k+0)-s2->at(5+4*k+0))*(s->at(5+4*k+0)-s2->at(5+4*k+0))+
								(s->at(5+4*k+1)-s2->at(5+4*k+1))*(s->at(5+4*k+1)-s2->at(5+4*k+1));
					}
					return sqrt(value)/velocity_factor;
				};
			}
			/*
			else if (plant_type == "treaded_vehicle" || plant_type == "treaded_vehicle_fo" || plant_type == "racecar")
			{
				h=[this,state_space](const space_point_t& s, const space_point_t& s2)
				{
					std::vector<double> x;
					state_space->copy_vector_from_point(x,s);
					x.at(0) -= s2->at(0);
					x.at(1) -= s2->at(1);
					x.at(2) = norm_angle_pi(x.at(2)-s2->at(2));
					return bm_metric_cost(x);
				};
			}
			*/
			else
			{
				h = [this,velocity_factor](const space_point_t& s, const space_point_t& s2)
				{
					return distance_function(s,s2)/velocity_factor;
				};
			}
		}
		else
		{
			prx_throw("Unknown cost function type in configurable_functions_t "<<cost_function_type);
		}


	}
}
