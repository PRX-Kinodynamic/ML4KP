#include "prx/planning/planners/sst.hpp"

namespace prx
{

	sst_t::sst_t(const std::string& new_name) : rrt_t(new_name)
	{
		metric = nullptr;
		witnesses = nullptr;
		planner_name = "SST";
	}
	sst_t::~sst_t()
	{
		_reset();
	}
	void sst_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		rrt_t::_link_and_setup_spec(spec);
		//reset is always called before this
		sst_spec = dynamic_cast<sst_specification_t*>(spec);
		witnesses = new graph_nearest_neighbors_t(distance_function);
		prx_assert(sst_spec!=nullptr,"SST received an incorrect specification.");
		prx_assert(sst_spec->blossom_number==1,"SST uses a blossom number of 1!");
	}
	bool sst_t::_preprocess()
	{
		tree.allocate_memory<sst_node_t,rrt_edge_t>(1000);
		witness_tree.allocate_memory<witness_node_t,rrt_edge_t>(1000);
		return true;
	}
	bool sst_t::_link_and_setup_query(planner_query_t* query)
	{
		rrt_query = dynamic_cast<rrt_query_t*>(query);
		prx_assert(rrt_query!=nullptr,"SST received an incorrect query type.");
		sst_query = dynamic_cast<sst_query_t*>(query);
		prx_assert(sst_query!=nullptr,"SST received an incorrect query type.");
		if(tree.num_vertices()==0 || !state_space->equal_points(tree.get_vertex_as<rrt_node_t>(start_vertex)->point,rrt_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			tree.clear();
			witnesses->clear();
			witness_tree.clear();
			
			start_vertex = tree.add_vertex<sst_node_t,rrt_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = tree.get_vertex_as<sst_node_t>(start_vertex);
			start_node->point = state_space->clone_point(rrt_query->start_state);
			start_node->cost_to_come = 0;
			metric->add_node(start_node.get());

			auto closest_witness = witness_tree.get_vertex_as<witness_node_t>(find_closest_witness(start_node->point));
			closest_witness->set = true;
			closest_witness->rep_index = start_vertex;

			start_node->witness_index = closest_witness->get_index();
		}
		timer.reset();
		iteration_count=0;
		current_solution=0;
		current_solution_iters=0;
		current_solution_time=0;
		return true;
	}
	void sst_t::_resolve_query(condition_check_t* condition)
	{
		//run for a certain amount of time
		do
		{
			//sample state
			sample_state(sample_point);
			//find closest
			auto prox_nodes = metric->radius_and_closest_query(sample_point,sst_spec->delta_near);
			std::vector<sst_node_t*> nodes;
			std::transform(prox_nodes.begin(),prox_nodes.end(),std::back_inserter(nodes),[]
			(proximity_node_t* prox_node)
			{
				return static_cast<sst_node_t*>(prox_node);
			});

			auto closest_node = *std::min_element(nodes.begin(),nodes.end(),[](sst_node_t* a, sst_node_t* b)
			{
				return a->cost_to_come < b->cost_to_come;
			});
			//expand			
			std::vector<plan_t*> plans;
			std::vector<trajectory_t*> trajs;
			expand(closest_node->point,plans,trajs,rrt_spec->blossom_number,false);
			plan_t plan(*plans.front());
			trajectory_t traj(*trajs.front());
			double edge_cost = cost_function(traj,plan);

			//bnb check
			if((goal_vertex==start_vertex || closest_node->cost_to_come + edge_cost < current_solution))
			{
				//check for closest witness
				auto closest_witness_index = find_closest_witness(traj.back());
				auto closest_witness = get_witness(closest_witness_index);

				//collision check
				if( (!closest_witness->set || get_vertex(closest_witness->rep_index)->cost_to_come > closest_node->cost_to_come + edge_cost) &&
					valid_check(traj))
				{
					//add node
					auto node_index = tree.add_vertex<sst_node_t,rrt_edge_t>();
					auto new_tree_node = tree.get_vertex_as<sst_node_t>(node_index);
					new_tree_node->point = state_space->clone_point(traj.back());
					edge_index_t edge_index = tree.add_edge(closest_node->get_index(),node_index);
					auto new_edge = tree.get_edge_as<rrt_edge_t>(edge_index);
					new_edge->plan = std::make_shared<plan_t>(plan);
					new_edge->traj = std::make_shared<trajectory_t>(traj);
					new_edge->edge_cost = edge_cost;
					new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;
					update_goal(node_index,condition);

					if(closest_witness->set)
					{
						//remove the node that was previously there
	                    if(!get_vertex(closest_witness->rep_index)->bridge)
	                    {
	                        metric->remove_node(get_vertex(closest_witness->rep_index));
	                        get_vertex(closest_witness->rep_index)->bridge = true;
	                    }
	                    node_index_t iter = closest_witness->rep_index;
	                    while( is_leaf(iter) && get_vertex(iter)->bridge && !is_best_goal(iter))
	                    {
	                        node_index_t next = get_vertex(iter)->get_parent();
	                        remove_leaf(iter);
	                        iter = next;
	                    } 
					}
					closest_witness->set = true;
					closest_witness->rep_index = node_index;
					new_tree_node->witness_index = closest_witness_index;
					new_tree_node->bridge = false;
					metric->add_node(new_tree_node.get());

				}
			}
			iteration_count++;
		}
		while(!condition->check());
	}

	void sst_t::_reset()
	{
		//clear the stuff
		tree.purge();
		witness_tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}
		if(witnesses!=nullptr)
		{
			delete witnesses;
			witnesses = nullptr;
		}

	}

	node_index_t sst_t::find_closest_witness(space_point_t s)
	{
		node_index_t witness_index;
		if(witnesses->get_nr_nodes()!=0)
		{
			witness_index = ((sst_node_t*)(witnesses->single_query(s)))->get_index();
		}

		if(witnesses->get_nr_nodes()==0 || distance_function(witness_tree.get_vertex_as<witness_node_t>(witness_index)->point,s) > sst_spec->delta_drain )
		{
			//create a new sample
			witness_index = witness_tree.add_vertex<witness_node_t,rrt_edge_t>();
			auto witness_node = witness_tree.get_vertex_as<witness_node_t>(witness_index);
			witness_node->point = state_space->clone_point(s);
			witnesses->add_node(witness_node.get());
		}
		return witness_index;
	}

	void sst_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
	{
		auto node = get_vertex(v);
		const bool res = delete_flag || node->cost_to_come > cost_bound;
		std::list<node_index_t> children = node->get_children();
		for(auto child : children)
		{
			bnb(child,cost_bound,res);
		}
		if(res && is_leaf(v))
		{

			//remove the node that was previously there
			if(!node->bridge)
			{
				metric->remove_node(node);
				node->bridge = true;
			}
			node_index_t witness = node->witness_index;
			get_witness(witness)->set = false;

			//remove the node
			tree.remove_vertex(v);
		}
	}

}