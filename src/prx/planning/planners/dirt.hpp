#pragma once

#include "prx/planning/planners/rrt.hpp"

namespace prx
{

	class dirt_node_t : public rrt_node_t
	{
	public:
		dirt_node_t() : rrt_node_t()
		{
			bridge = false;
			is_blossom_expand_done = false;
			random_expand = false;
		}
		virtual ~dirt_node_t()
		{
			for(auto eg : edge_generators)
			{
				if(eg.first!=nullptr)
					delete eg.first;
				if(eg.second!=nullptr)
					delete eg.second;
			}
			edge_generators.clear();
			is_blossom_expand_done = false;
			random_expand = false;
		}

		double cost_to_go;

		bool bridge;

		double dir_radius;

		int blossom_number;

		bool is_blossom_expand_done, random_expand;

		std::vector<std::pair<plan_t*,trajectory_t*>> edge_generators;

		std::vector<int> indices;

		double checkpoint_time;
		bool is_safety_node;
	};

	class dirt_specification_t : public rrt_specification_t
	{
	public:
		dirt_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) : rrt_specification_t(sg,cg)
		{
			h = [this](const space_point_t& s, const space_point_t& s2)
			{
				return default_heuristic_function(s, s2, distance_function);
			};
			obstacle_distance_function = [cg,this](const space_point_t& s)
			{
				return default_obstacle_distance_function(s,state_space,cg);
			};
			blossom_number = 5;
			use_pruning = true;
		}
		virtual ~dirt_specification_t(){}

		int blossom_number;

		bool use_pruning;

		double replanning_cycle;
		int order;

		heuristic_function_t h;
		obstacle_distance_function_t obstacle_distance_function;
	};

	class dirt_query_t : public rrt_query_t
	{
	public:
		dirt_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space,control_space)
		{
		}
		virtual ~dirt_query_t(){}

	};

	class dirt_t : public rrt_t
	{
	public:
		dirt_t(const std::string& new_name);
		virtual ~dirt_t();

		std::vector<long unsigned> random_edges_counter, blossom_edges_counter;

	protected:

		virtual void update_goal(node_index_t node_index) override;

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _reset() override;

		virtual std::vector<double> get_statistics() override;

		dirt_specification_t* dirt_spec;
		dirt_query_t* dirt_query;


		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;


	private:

		int replanning_iteration;
		double ri_step;	// Time during a replanning cycle

		heuristic_function_t h;
		expand_t expand;

		double max_radius;
		bool child_extension;
		node_index_t previous_child;

		node_index_t add_edge_to_tree(std::pair<plan_t*, trajectory_t*> eg,
		dirt_node_t* closest_node,
		std::vector<dirt_node_t*> dir_updates,
		double new_node_dir_radius
		);

		dirt_node_t* get_vertex(node_index_t v) const
		{
			return tree.get_vertex_as<dirt_node_t>(v).get();
		}

		bool is_leaf(node_index_t v)
		{
			return (get_vertex(v)->get_children().empty());
		}

		void remove_leaf(node_index_t v)
		{
			prx_assert(is_leaf(v),"Trying to remove a tree node that is not a leaf!");

			if(!get_vertex(v)->bridge)
			{
				metric->remove_node(get_vertex(v));
				get_vertex(v)->bridge = true;
			}
			tree.remove_vertex(v);
		}

		bool is_best_goal(node_index_t v) const
		{
			node_index_t new_v = goal_vertex;
			while(get_vertex(new_v)->get_parent()!=new_v)
			{
				if(new_v == v)
					return true;
				new_v = get_vertex(new_v)->get_parent();
			}
			return false;

		}
	};
}
