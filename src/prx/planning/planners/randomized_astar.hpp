#pragma once

#include "prx/planning/planners/rrt.hpp"
#include "prx/utilities/data_structures/open_set.hpp"


namespace prx
{        

	class rastar_node_t : public rrt_node_t
	{
	public:
		rastar_node_t() : rrt_node_t()
		{
			expanded = false;
			astar_node = nullptr;
		}
		virtual ~rastar_node_t()
		{
			if(astar_node != nullptr)
			{
				delete astar_node;
				astar_node = nullptr;
			}
		}
		double cost_to_go;
		bool expanded;

		astar_node_t* astar_node;
	};

	class rastar_specification_t : public rrt_specification_t
	{
	public:
		rastar_specification_t(std::shared_ptr<system_group_t> sg,std::shared_ptr<collision_group_t> cg) : rrt_specification_t(sg,cg)
		{
			h = [this](const space_point_t& s, const space_point_t& s2)
			{
				return default_heuristic_function(s, s2, distance_function);
			};
			
			blossom_number = 5;
			delta_prune = 0;

			// TODO: Is this working or should we remove it?
			rrt_select = false;
		}
		virtual ~rastar_specification_t(){}

		double delta_prune;

		int blossom_number;

		bool rrt_select;

		heuristic_function_t h;
	};

	class rastar_query_t : public rrt_query_t
	{
	public:
		rastar_query_t(space_t* state_space, space_t* control_space) : rrt_query_t(state_space,control_space)
		{
		}
		virtual ~rastar_query_t(){}

	};

	class randomized_astar_t : public rrt_t
	{
	public:
		randomized_astar_t(const std::string& new_name);
		virtual ~randomized_astar_t();

	protected:
		open_set_t open_set;

		virtual void _link_and_setup_spec(planner_specification_t* spec) override;
		virtual bool _preprocess() override;
		virtual bool _link_and_setup_query(planner_query_t* query) override;
		virtual void _resolve_query(condition_check_t* condition) override;
		virtual void _reset() override;

		virtual std::vector<double> get_statistics() override;

		rastar_specification_t* rastar_spec;
		rastar_query_t* rastar_query;
		
		virtual void bnb(node_index_t v, double cost_bound, bool delete_flag = false) override;

		std::vector<long unsigned> random_edges_counter, blossom_edges_counter;

	private:

		heuristic_function_t h;
		expand_t expand;

		rastar_node_t* get_vertex(node_index_t v) const
		{
			return tree.get_vertex_as<rastar_node_t>(v).get();
		}

		bool is_leaf(node_index_t v)
		{
			return (get_vertex(v)->get_children().empty());
		}

		void remove_leaf(node_index_t v)
		{
			prx_assert(is_leaf(v),"Trying to remove a tree node that is not a leaf!");
			metric->remove_node(get_vertex(v));
			tree.remove_vertex(v);
		}
	};
}
