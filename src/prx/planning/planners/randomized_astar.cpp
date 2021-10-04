#include "prx/planning/planners/randomized_astar.hpp"

namespace prx
{

	randomized_astar_t::randomized_astar_t(const std::string& new_name) : rrt_t(new_name)
	{
		metric = nullptr;
	}
	randomized_astar_t::~randomized_astar_t()
	{
		_reset();
	}
	void randomized_astar_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		rrt_t::_link_and_setup_spec(spec);
		//reset is always called before this
		rastar_spec = dynamic_cast<rastar_specification_t*>(spec);
		prx_assert(rastar_spec!=nullptr,"Randomized A* received an incorrect specification.");

		expand = rastar_spec->expand;
		h = rastar_spec->h;

		// This is used only in the case of an RRT-like selection.
		sample_point = rastar_spec->state_space->make_point();
	}
	bool randomized_astar_t::_preprocess()
	{
		tree.allocate_memory<rastar_node_t,rrt_edge_t>(1000);
		return true;
	}
	bool randomized_astar_t::_link_and_setup_query(planner_query_t* query)
	{
		rrt_query = dynamic_cast<rrt_query_t*>(query);
		prx_assert(rrt_query!=nullptr,"Randomized A* received an incorrect query type.");
		rastar_query = dynamic_cast<rastar_query_t*>(query);
		prx_assert(rastar_query!=nullptr,"Randomized A* received an incorrect query type.");
		if(tree.num_vertices()==0 || !state_space->equal_points(tree.get_vertex_as<rrt_node_t>(start_vertex)->point,rrt_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			tree.clear();
			open_set.clear();
			
			start_vertex = tree.add_vertex<rastar_node_t,rrt_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = tree.get_vertex_as<rastar_node_t>(start_vertex);
			start_node->point = state_space->clone_point(rrt_query->start_state);
			start_node->cost_to_come = 0;
			start_node->cost_to_go = h(start_node->point,rastar_query->goal_state);
			metric->add_node(start_node.get());
			auto astar_node = new astar_node_t(start_vertex,start_node->cost_to_come,start_node->cost_to_go);
			start_node->astar_node = astar_node;
			open_set.insert(astar_node);
		}
		timer.reset();
		iteration_count=0;
		current_solution=0;
		current_solution_iters=0;
		current_solution_time=0;

		random_edges_counter = {0,0,0,0};
		blossom_edges_counter = {0,0,0,0};

		return true;
	}
	void randomized_astar_t::_resolve_query(condition_check_t* condition)
	{
		//run for a certain amount of time
		do
		{
			if(open_set.empty())
			{
				continue;
			}

			// This was to deal with the auto. Maybe it should be moved outside the do-while.
			rastar_node_t* closest_node;

			if (!rastar_spec->rrt_select)
			{
				auto next_astar_node = open_set.remove_min();
				closest_node = get_vertex(next_astar_node->vertex);
			}
			else
			{
				sample_state(sample_point);
				closest_node = static_cast<rastar_node_t*>(metric->single_query(sample_point));
			}

			if (!closest_node->expanded)
			{
				std::vector<plan_t*> plans;
				std::vector<trajectory_t*> trajs;
				expand(closest_node->point,plans,trajs,rastar_spec->blossom_number,true);  

				std::pair<plan_t*,trajectory_t*> eg = std::make_pair(nullptr,nullptr);
				double edge_cost;
				double end_heuristic;

				while(plans.size()>0)
				{
					eg.first = plans.back();
					eg.second = trajs.back();
					plans.pop_back();
					trajs.pop_back();
					edge_cost = cost_function(*eg.second,*eg.first);
					end_heuristic = h(eg.second->back(),rastar_query->goal_state);


					//bnb
					if((goal_vertex!=start_vertex && closest_node->cost_to_come + edge_cost + closest_node->cost_to_go > current_solution))
					{
						delete eg.first;
						delete eg.second;
						eg = std::make_pair(nullptr,nullptr);
						continue;
					}

					auto prune_check_node = static_cast<rastar_node_t*>(metric->single_query(eg.second->back()));
					if(distance_function(prune_check_node->point,eg.second->back()) < rastar_spec->delta_prune)
					{
						delete eg.first;
						delete eg.second;
						eg = std::make_pair(nullptr,nullptr);
						continue;
					}

					//validity check
					if(!valid_check(*eg.second))
					{
						delete eg.first;
						delete eg.second;
						eg = std::make_pair(nullptr,nullptr);
						continue;
					}

					//add the node
					auto node_index = tree.add_vertex<rastar_node_t,rrt_edge_t>();
					auto new_tree_node = tree.get_vertex_as<rastar_node_t>(node_index);
					new_tree_node->point = state_space->clone_point(eg.second->back());
					edge_index_t edge_index = tree.add_edge(closest_node->get_index(),node_index);
					auto new_edge = tree.get_edge_as<rrt_edge_t>(edge_index);
					new_edge->plan = std::make_shared<plan_t>(*eg.first);
					new_edge->traj = std::make_shared<trajectory_t>(*eg.second);
					delete eg.first;
					delete eg.second;
					new_edge->edge_cost = edge_cost;
					new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;
					new_tree_node->cost_to_go = end_heuristic;
					
					update_goal(node_index,condition);
					metric->add_node(new_tree_node.get());

					// Don't spend time on insertion to heap if you're never gonna use it anyways.
					if (!rastar_spec->rrt_select)
					{
						auto astar_node = new astar_node_t(node_index,new_tree_node->cost_to_come,new_tree_node->cost_to_go);
						new_tree_node->astar_node = astar_node;
						open_set.insert(astar_node);
					}
					closest_node->expanded = true;
				}
			}
			iteration_count++;
		}
		while(!condition->check());
	}

	std::vector<double> randomized_astar_t::get_statistics()
	{
		std::vector<double> rrt_statistics = rrt_t::get_statistics();
		return rrt_statistics;
	}

	void randomized_astar_t::_reset()
	{
		//clear the stuff
		tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}

	void randomized_astar_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
	{
		if(v==start_vertex)
		{
			open_set.clear();
		}
		auto node = get_vertex(v);
		bool res = delete_flag || node->cost_to_come + node->cost_to_go > cost_bound;
		if(v == goal_vertex)
			res = false;
		std::list<node_index_t> children = node->get_children();
		for(auto child : children)
		{
			bnb(child,cost_bound,res);
		}
		if(res && is_leaf(v))
		{
			delete node->astar_node;
			node->astar_node = nullptr;
			//remove the node that was previously there
			metric->remove_node(node);
			//remove the node
			tree.remove_vertex(v);
		}
		if(v==start_vertex)
		{
			auto iters = tree.vertices();
			for(auto iter = iters.first; iter!=iters.second; iter++)
			{
				if(static_cast<rastar_node_t*>(iter->get())->astar_node!=nullptr)
				{
					open_set.insert(static_cast<rastar_node_t*>(iter->get())->astar_node);
				}
			}
		}
	}
}