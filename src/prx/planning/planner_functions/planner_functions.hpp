#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

namespace prx
{
	typedef std::function<void (space_point_t&)> sample_state_t;
	typedef std::function<void (plan_t&, space_point_t)> sample_plan_t;
	typedef std::function<bool (trajectory_t&)> valid_trajectory_t;
	typedef std::function<bool (space_point_t, plan_t*, trajectory_t*)> valid_stop_t;
	typedef std::function<bool (space_point_t&)> valid_state_t;
	typedef std::function<void (space_point_t&, plan_t&, trajectory_t& )> propagate_t;
	typedef std::function<void (space_point_t&, std::vector<plan_t*>&, std::vector<trajectory_t*>&, int bn, bool blossom_expand)> expand_t;

	typedef std::function<double (const trajectory_t&, const plan_t&)> cost_function_t;
	typedef std::function<double (const space_point_t&, const space_point_t&)> heuristic_function_t;
	typedef std::function<collision_group_t::pqp_distance_t (const space_point_t&)> obstacle_distance_function_t;
	
	// GLC-related functions
	typedef std::function<double (const int&)> eta_function_t;
	typedef std::function<int (const int&)> horizon_function_t;
	typedef std::function<std::set<std::pair<std::shared_ptr<plan_t>, std::shared_ptr<trajectory_t>>> (space_point_t&)> expand_set_t;

	class planner_functions_t
	{
	public:
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_state_t valid_state;
		valid_trajectory_t valid_check;
		valid_stop_t valid_stop_check;
		propagate_t propagate;
		expand_t expand;
		cost_function_t cost_function;
		heuristic_function_t h;
		distance_function_t distance_function;
		horizon_function_t horizon_function;
		eta_function_t eta_function;
		obstacle_distance_function_t obstacle_distance_function;

		planner_functions_t(std::string context_name, world_model_context context, param_loader pl);
	protected:
		planner_functions_t(){}
	};

	void default_sample_state(space_point_t&,space_t*);

	void default_sample_plan(plan_t&, space_t*, int min_steps, int max_steps);

	bool default_valid_trajectory(trajectory_t&,space_t*,std::shared_ptr<collision_group_t>);

	bool default_valid_stop(space_point_t start_state,
					plan_t* stopping_plan,
					trajectory_t* stopping_traj,
					std::shared_ptr<system_group_t> sg,
					std::shared_ptr<collision_group_t> cg
					);	// replanning: inevitable collision state check

	bool default_valid_state(space_point_t&,space_t*,std::shared_ptr<collision_group_t>);

	void default_propagate(space_point_t&, plan_t&, trajectory_t&,std::shared_ptr<system_group_t>);

	void default_expand(space_point_t&, std::vector<plan_t*>&, std::vector<trajectory_t*>&, int bn, std::shared_ptr<system_group_t>, sample_plan_t sp, propagate_t prop);


	double default_cost_function(const trajectory_t&, const plan_t&);

	double default_heuristic_function(const space_point_t&, const space_point_t&, distance_function_t d);

	collision_group_t::pqp_distance_t default_obstacle_distance_function(const space_point_t&,space_t*,std::shared_ptr<collision_group_t>);

	double default_eta_function(const int&);

	int default_horizon_function(const int&);

	std::set<std::pair<std::shared_ptr<plan_t>, std::shared_ptr<trajectory_t>>> default_expand_set(
		space_point_t& start_state, std::set<std::pair<space_point_t, double>> pts_time_set, 
							std::shared_ptr<system_group_t> sg, propagate_t prop);

}
