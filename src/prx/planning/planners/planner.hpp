#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/condition_check.hpp"

namespace prx
{
	/**
     * @brief <b>A class that specified the parameters of a planner.</b>
     * @authors Zakary Littlefield
     */
	typedef std::function<bool (space_point_t&)> goal_check_t;
	class planner_specification_t
	{
	public:
		planner_specification_t(){}
		virtual ~planner_specification_t(){}
	};
	class planner_query_t
	{
	public:
		planner_query_t(space_t* state_space, space_t* control_space):solution_traj(state_space),solution_plan(control_space){}
		virtual ~planner_query_t(){}

		virtual void clear_outputs()
		{
			solution_traj.clear();
			solution_plan.clear();
			solution_cost = 0;
			tree_visualization.clear();
		}

		//inputs
		space_point_t start_state;
		space_point_t goal_state;
		goal_check_t goal_check;
		double goal_region_radius;
		bool get_visualization;

		//outputs
		trajectory_t solution_traj;
		plan_t solution_plan;
		double solution_cost;
		std::vector<trajectory_t> tree_visualization;
	};

	/**
	 *    +------------->Construction
	 *	  |                   +
	 *	  |                   |
	 *	  |    +-----------+  |
	 *	  |    |           v  v
	 *	  |  +-^------Link and Setup Spec
	 *	  |  | |              +   + ^
	 *	  |  | |              |   +-+
	 *	  |  | |              |
	 *	  +  | +^---------+   v
	 *	Reset<-----------+Preprocess
	 *	  ^    |              +  + ^
	 *	  |    |              |  +-+
	 *	  |    |              |
	 *	  |    +^----+        v
	 *	  +^--------+Link and Setup Query
	 *	  |    |              +   + ^ ^ ^
	 *	  |    |              |   +-+ | |
	 *	  |    |              |       | |
	 *	  |    +^--------+    ^       | |
	 *	  +^------------+Resolve Query+ |
	 *	  |    |              |   + ^   |
	 *	  |    |              |   +-+   |
	 *	  |    |              |         |
	 *	  |    +---------+    v         |
	 *	  +-------------+Finalize Query++
	 *
	 */
	class planner_t
	{
	public:
		planner_t(const std::string& new_name);
		virtual ~planner_t();

		/**
		*	Previous Function: Any function (Reset will be called)
		*/
		void link_and_setup_spec(planner_specification_t* spec);
		// void link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec);

		/**
		*	Previous Function: Link spec or preprocess
		*/
		bool preprocess();

		/**
		*	Previous Function: Preprocess, Query functions
		*/
		bool link_and_setup_query(planner_query_t* query);
		// bool link_and_setup_query_shared(std::shared_ptr<planner_query_t> query);

		/**
		*	Previous Function: Link and resolve query
		*/
		void resolve_query(condition_check_t* condition);

		/**
		*	Previous Function: Resolve Query
		*/
		void fulfill_query();

		/**
		*	Previous Function: Any function.
		*/
		void reset();

		virtual std::vector<std::string> get_statistics_header()
		{
			return {};
		}

		virtual std::vector<double> get_statistics()
		{
			return {};
		}

	protected:
		virtual void _link_and_setup_spec(planner_specification_t* spec)=0;
		virtual bool _preprocess()=0;
		virtual bool _link_and_setup_query(planner_query_t* query)=0;
		virtual void _resolve_query(condition_check_t* condition)=0;
		virtual void _fulfill_query()=0;
		virtual void _reset()=0;

		// virtual void _link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec)=0;
		// virtual bool _link_and_setup_query_shared(std::shared_ptr<planner_query_t> query)=0;
		std::string planner_name;

	private:

		std::string get_current_stage_name();

		enum class planner_stage_t
		{
			CONSTRUCTION, LINK_SPECIFICATION, PREPROCESS, LINK_QUERY, RESOLVE_QUERY, FULFILL_QUERY
		} planner_state;


	};
}
