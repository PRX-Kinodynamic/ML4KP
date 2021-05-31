#pragma once

#include "prx/planning/planners/rrt.hpp"

namespace prx
{        

	class sst_node_t : public rrt_node_t
	{
	public:
		sst_node_t() : rrt_node_t()
		{
			bridge = false;
		}
		virtual ~sst_node_t(){}

		bool bridge;

		node_index_t witness_index;
	};


	class witness_node_t : public tree_node_t
	{
	public:
		witness_node_t() : tree_node_t()
		{
			set = false;
		}
		virtual ~witness_node_t(){}
		node_index_t rep_index;
		bool set;
	};

	class sst_specification_t : public rrt_specification_t
	{
	public:
		sst_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) : rrt_specification_t(sg,cg)
		{
		}
		virtual ~sst_specification_t(){}

		double delta_near;
		double delta_drain;
	};

	class sst_query_t : public rrt_query_t
	{
	public:
		sst_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space,control_space)
		{
		}
		virtual ~sst_query_t(){}

	};

	class sst_t : public rrt_t
	{
	public:
		sst_t(const std::string& new_name);
		virtual ~sst_t();

	protected:

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _reset() override;

		node_index_t find_closest_witness(space_point_t s);

		graph_nearest_neighbors_t* witnesses;
		tree_t witness_tree;

		sst_specification_t* sst_spec;
		sst_query_t* sst_query;

		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;

	private:
		sst_node_t* get_vertex(node_index_t v) const
		{
			return tree.get_vertex_as<sst_node_t>(v).get();
		}
		witness_node_t* get_witness(node_index_t v) const
		{
			return witness_tree.get_vertex_as<witness_node_t>(v).get();
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
