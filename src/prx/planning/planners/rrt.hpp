#pragma once

#include "prx/planning/planners/planner.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/utilities/defs.hpp"
#include "prx/utilities/general/timer.hpp"
#include "prx/utilities/data_structures/sigma.hpp"

#define PLANNER_NAME "RRT"
namespace prx
{

	class rrt_node_t : public tree_node_t
	{
	public:
		rrt_node_t()
		{
			cost_to_come = 0;
			duration = 0;
		}
		virtual ~rrt_node_t(){}

		double cost_to_come;
		double duration;
	};

	class rrt_edge_t : public tree_edge_t
	{
	public:
		rrt_edge_t()
		{
			edge_cost = 0;
		}
		virtual ~rrt_edge_t(){}

		std::shared_ptr<plan_t> plan;
		std::shared_ptr<trajectory_t> traj;
		double edge_cost;
	};

	class rrt_specification_t : public planner_specification_t
	{
	public:
		rrt_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg)
		{
			bnb = true;
			use_replanning = false;
			state_space = sg->get_state_space();
			control_space = sg->get_control_space();
			double multiplier = simulation_step >= 1 ? simulation_step : 1./simulation_step;
			min_control_steps = 0.5*multiplier;
			max_control_steps = 5*multiplier;

			cost_function = [](const trajectory_t& t, const plan_t& plan)
			{
				return default_cost_function(t,plan);
			};
			distance_function = [](const space_point_t& s1, const space_point_t& s2)
			{
				return space_t::euclidean_2d(s1, s2);
			};
			sample_state = [this](space_point_t& s)
			{
				default_sample_state(s,state_space);
			};
			sample_plan = [this](plan_t& p, space_point_t pose)
			{
				// default_sample_plan(p,control_space,100,400);
				// Changed to time
				default_sample_plan(p,control_space,min_control_steps,max_control_steps);
			};
			valid_state = [this,cg](space_point_t& s)
			{
				return default_valid_state(s, state_space,cg);
			};
			valid_check = [this,cg](trajectory_t& traj)
			{
				return default_valid_trajectory(traj, state_space,cg);
			};
			valid_stop_check = [this, sg, cg](space_point_t start_state,
									plan_t* stopping_plan,
									trajectory_t* stopping_traj
									)
			{
				return default_valid_stop(start_state, stopping_plan, stopping_traj, sg, cg);
			};
			propagate = [sg](space_point_t& start_state, plan_t& plan, trajectory_t& out_traj)
			{
				default_propagate(start_state,plan,out_traj,sg);
			};
			expand = [sg,this](space_point_t& s, std::vector<plan_t*>& plans, std::vector<trajectory_t*>& trajs, int bn, bool blossom_expand)
			{
				default_expand(s, plans, trajs, bn, sg, sample_plan, propagate);
			};

			blossom_number = 1;
		}
		virtual ~rrt_specification_t(){}

		cost_function_t cost_function;
		distance_function_t distance_function;
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_trajectory_t valid_check;
		valid_stop_t valid_stop_check;
		valid_state_t valid_state;
		expand_t expand;
		propagate_t propagate;

		space_t* state_space;
		space_t* control_space;

		bool bnb;
		bool use_replanning;

		int min_control_steps;
		int max_control_steps;
		int blossom_number;

	};

	class rrt_query_t : public planner_query_t
	{
	public:
		rrt_query_t(space_t* state_space, space_t* control_space) : planner_query_t(state_space,control_space)
		{
			clear_outputs();

			goal_check = [&](space_point_t s)
			{
				return space_t::euclidean_2d(s, goal_state) < goal_region_radius;
			};

		}
		virtual ~rrt_query_t(){}
		double goal_region_radius;
	};

	class rrt_t : public planner_t
	{
	public:
		rrt_t(const std::string& new_name);
		virtual ~rrt_t();

		virtual void print_statistics();

		virtual std::vector<std::string> get_statistics_header() override;
		virtual std::vector<double> get_statistics() override;
	protected:

		virtual void update_goal(node_index_t node_index, condition_check_t* condition);

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _fulfill_query() override;
		virtual void _reset() override;


		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false);

		rrt_specification_t* rrt_spec;
		rrt_query_t* rrt_query;

		// virtual void _link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec) override;
		// virtual bool _link_and_setup_query_shared(std::shared_ptr<planner_query_t> query) override;

		std::string planner_name;
		
		node_index_t start_vertex;
		node_index_t goal_vertex;

		distance_function_t distance_function;
		cost_function_t cost_function;
		sample_state_t sample_state;
		sample_plan_t sample_plan;
		valid_trajectory_t valid_check;
		valid_stop_t valid_stop_check;
		expand_t expand;
		propagate_t propagate;

		tree_t tree;
		graph_nearest_neighbors_t* metric;

		space_t* state_space;
		space_t* control_space;

		space_point_t sample_point;

		long unsigned iteration_count;
		timer_t timer;

		double current_solution;
		long unsigned current_solution_iters;
		double current_solution_time;

		bool use_replanning;

		int print_statistics_count;

	};
}
