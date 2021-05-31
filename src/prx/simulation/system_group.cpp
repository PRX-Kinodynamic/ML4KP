
#include "prx/simulation/system_group.hpp"

#include <algorithm>

namespace prx
{
	system_group_t::system_group_t(const std::vector<system_ptr_t>& sys_group)
	{
		group = sys_group;
		//construct the state and control spaces after checking for overlap
		std::vector<system_ptr_t> to_remove;
		for(auto g1 : group)
		{
			for(auto g2 : group)
			{
				if(g1!=g2 && ( is_child_system(g1,g2) || is_child_system(g2,g1) ) )
				{
					if(g1->get_pathname().length()>g2->get_pathname().length())
						to_remove.push_back(g1);
					else
						to_remove.push_back(g2);
				}

			}
		}
		auto it = std::unique (to_remove.begin(), to_remove.end());
		to_remove.resize( std::distance(to_remove.begin(),it) );
		group.erase( std::remove_if( std::begin(group),std::end(group),
								[&](system_ptr_t x)
								{
									return std::find(std::begin(to_remove),std::end(to_remove),x)!=std::end(to_remove);
								}),
					std::end(group) );

		//compose the state spaces and control spaces
		std::vector<const space_t*> state_spaces;
		std::vector<const space_t*> control_spaces;
		for(auto g1 : group)
		{
			state_spaces.push_back(g1->get_state_space());
			control_spaces.push_back(g1->get_control_space());
		}
		state_space = new space_t(state_spaces);
		control_space = new space_t(control_spaces);
	}

	system_group_t::~system_group_t()
	{
		delete state_space;
		delete control_space;
		group.clear();
	}

	void system_group_t::propagate(space_point_t start_state, const plan_t& plan, space_point_t result)
	{
		state_space->copy_from_point(start_state);
		propagate_step p_step;
		
		for(const plan_step_t& step : plan)
		{
			int steps = (int)((step.duration / simulation_step) + .1);
			int i = 0;
			if( steps > 0 )
			{
				for( ; i < steps; i++ )
				{
					if (i == 0) p_step = propagate_step::FIRST_STEP;
					else if (i > 0 && i < steps-1) p_step = propagate_step::MIDDLE_STEP;
					else p_step = propagate_step::FINAL_STEP;

					propagate_once(step.control,p_step);
				}
			}
		}
		state_space->copy_to_point(result);
	}

	void system_group_t::propagate(space_point_t start_state, const plan_t& plan, trajectory_t& traj)
	{
		state_space->copy_from_point(start_state);
		propagate_step p_step;

		traj.clear();
		traj.copy_onto_back(state_space);
		for(const plan_step_t& step : plan)
		{
			int steps = (int)((step.duration / simulation_step) + .1);
			int i = 0;
			if( steps > 0 )
			{
				for( ; i < steps; i++ )
				{
					if (i == 0) p_step = propagate_step::FIRST_STEP;
					else if (i > 0 && i < steps-1) p_step = propagate_step::MIDDLE_STEP;
					else p_step = propagate_step::FINAL_STEP;

					propagate_once(step.control,p_step);
					traj.copy_onto_back(state_space);
				}
			}
		}
	}

	void system_group_t::propagate_once(space_point_t control, propagate_step step)
	{
		control_space->copy_from_point(control);
		for(auto s : group)
		{
			s->compute_control();

			s->propagate(simulation_step, step);
		}
	}

	void system_group_t::compute_stopping_maneuver(space_point_t start_state, std::vector<double>& times, std::vector<double>& ctrls)
	{
		prx_assert(group.size() == 1, "[system_group_t::compute_stopping_maneuver] Expected group of size 1 but got "<<group.size());
		group[0]->compute_stopping_maneuver(start_state, times, ctrls);
	}
}
