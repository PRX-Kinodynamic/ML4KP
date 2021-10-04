#include "prx/planning/planners/rrt.hpp"

namespace prx
{
	rrt_t::rrt_t(const std::string& new_name) : planner_t(new_name)
	{
		metric = nullptr;
		planner_name = new_name;
	}
	rrt_t::~rrt_t()
	{
		_reset();
	}
	void rrt_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		//reset is always called before this
		rrt_spec = dynamic_cast<rrt_specification_t*>(spec);
		prx_assert(rrt_spec!=nullptr,"RRT received an incorrect specification.");
		distance_function = rrt_spec->distance_function;
		cost_function = rrt_spec->cost_function;
		sample_state = rrt_spec->sample_state;
		sample_plan = rrt_spec->sample_plan;
		valid_check = rrt_spec->valid_check;
		valid_stop_check = rrt_spec->valid_stop_check;
		propagate = rrt_spec->propagate;
		use_replanning = rrt_spec->use_replanning;
		expand = rrt_spec->expand;


		state_space = rrt_spec->state_space;
		sample_point = state_space->make_point();
		control_space = rrt_spec->control_space;
		metric = new graph_nearest_neighbors_t(distance_function);
		//we now have spaces and necessary functions
	}

	bool rrt_t::_preprocess()
	{
		tree.allocate_memory<rrt_node_t,rrt_edge_t>(1000);
		return true;
	}

	bool rrt_t::_link_and_setup_query(planner_query_t* query)
	{
		rrt_query = dynamic_cast<rrt_query_t*>(query);
		prx_assert(rrt_query!=nullptr,"RRT received an incorrect query type.");
		if(tree.num_vertices()==0 || !state_space->equal_points(tree.get_vertex_as<rrt_node_t>(start_vertex)->point,rrt_query->start_state) )
		{
			//clear existing data structure
			metric->clear();
			tree.clear();
			start_vertex = tree.add_vertex<rrt_node_t,rrt_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = tree.get_vertex_as<rrt_node_t>(start_vertex);
			start_node->point = state_space->clone_point(rrt_query->start_state);
			start_node->cost_to_come = 0;
			metric->add_node(start_node.get());

		}
		timer.reset();
		iteration_count=0;
		current_solution=0;
		current_solution_iters=0;
		current_solution_time=0;
		return true;
	}
	
	void rrt_t::_resolve_query(condition_check_t* condition)
	{
		bool glc_conds;
		double edge_cost;
		double new_cost;
		double new_duration;

		prx_warn_cond(rrt_spec->blossom_number==1 == 1, "RRT only uses a blossom number of 1")
		//run for a certain amount of time
		do
		{
			//sample state
			sample_state(sample_point);
			//find closest
			auto closest_node = static_cast<rrt_node_t*>(metric->single_query(sample_point));
			//expand			
			std::vector<plan_t*> plans;
			std::vector<trajectory_t*> trajs;
			expand(closest_node->point,plans,trajs,rrt_spec->blossom_number,false);
			plan_t plan(*plans.front());
			trajectory_t traj(*trajs.front());
			edge_cost = cost_function(traj,plan);

			new_cost = closest_node -> cost_to_come + edge_cost;
			new_duration = closest_node -> duration + plan.duration();
			//collision check && bnb && glc_conds
			if((goal_vertex==start_vertex || closest_node->cost_to_come + edge_cost < current_solution) 
				&& valid_check(traj))
			{
				//add node
				auto node_index = tree.add_vertex<rrt_node_t,rrt_edge_t>();
				auto new_tree_node = tree.get_vertex_as<rrt_node_t>(node_index);
				new_tree_node->point = state_space->clone_point(traj.back());
				metric->add_node(new_tree_node.get());
				edge_index_t edge_index = tree.add_edge(closest_node->get_index(),node_index);
				auto new_edge = tree.get_edge_as<rrt_edge_t>(edge_index);
				new_edge->plan = std::make_shared<plan_t>(plan);
				new_edge->traj = std::make_shared<trajectory_t>(traj);
				new_edge->edge_cost = edge_cost;
				new_tree_node->cost_to_come = closest_node->cost_to_come + new_edge->edge_cost;

				new_tree_node -> duration = new_duration;
				update_goal(node_index,condition);
			}
			iteration_count++;
		}
		while(!condition->check());
	}
	void rrt_t::_fulfill_query()
	{
		if(goal_vertex!=start_vertex && !use_replanning)
		{
			//backtrack to get the plan and trajectory
			rrt_query->solution_cost = tree.get_vertex_as<rrt_node_t>(goal_vertex)->cost_to_come;
			std::deque<node_index_t> node_indices;
			node_index_t current_index = goal_vertex;
			while(current_index!=start_vertex)
			{
				node_indices.push_front(current_index);
				current_index = tree[current_index]->get_parent();
			}

			rrt_query->solution_plan = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->plan;
			rrt_query->solution_traj = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->traj;

			for(int i=1;i<node_indices.size();i++)
			{
				rrt_query->solution_traj.resize(rrt_query->solution_traj.size()-1);
				rrt_query->solution_plan += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->plan;
				rrt_query->solution_traj += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->traj;
			}

		}
		else
		{
			rrt_query->solution_cost = 0;
		}
		if (use_replanning&&false)
		{
			std::deque<node_index_t> node_indices;
			node_index_t current_index;
			if (goal_vertex!=start_vertex)
			{
				rrt_query->solution_cost = tree.get_vertex_as<rrt_node_t>(goal_vertex)->cost_to_come;
				current_index = goal_vertex;
				while(current_index!=start_vertex)
				{
					node_indices.push_front(current_index);
					current_index = tree[current_index]->get_parent();
				}

			}
			current_index = start_vertex;
			while(current_index!=0)
			{
				node_indices.push_front(current_index);
				current_index = tree[current_index]->get_parent();
			}
			rrt_query->solution_plan = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->plan;
			rrt_query->solution_traj = *tree.get_edge_as<rrt_edge_t>(tree[node_indices[0]]->get_parent_edge())->traj;

			for(int i=1;i<node_indices.size();i++)
			{
				rrt_query->solution_traj.resize(rrt_query->solution_traj.size()-1);
				rrt_query->solution_plan += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->plan;
				rrt_query->solution_traj += *tree.get_edge_as<rrt_edge_t>(tree[node_indices[i]]->get_parent_edge())->traj;
			}
		}
		if(rrt_query->get_visualization)
		{
	        	auto iter_bounds = tree.edges();
	        	for(auto iter = iter_bounds.first; iter!=iter_bounds.second; iter++)
	        	{
				rrt_query->tree_visualization.push_back(*tree.get_edge_as<rrt_edge_t>((*iter)->get_index())->traj);
			}
		}
	}

	std::vector<std::string> rrt_t::get_statistics_header()
	{
		return {"time", "iters", "nodes", "solution_cost", "solution_time", "solution_iters"};
	}
	std::vector<double> rrt_t::get_statistics()
	{
		//time, iters, nodes, solution quality, first_time, first_iters, current_solution
		return {timer.measure(),
				static_cast<double>(iteration_count),
				static_cast<double>(metric->get_nr_nodes()),
				current_solution,
				current_solution_time,
				static_cast<double>(current_solution_iters)
				};
	}

	void rrt_t::_reset()
	{
		//clear the stuff
		tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}

	void rrt_t::update_goal(node_index_t node_index, condition_check_t* condition)
	{
		auto new_tree_node = tree.get_vertex_as<rrt_node_t>(node_index);
		// if(distance_function(rrt_query->goal_state,new_tree_node->point)<rrt_query->goal_region_radius)
		if(rrt_query->goal_check(new_tree_node->point))
		{
			if(goal_vertex==start_vertex || tree.get_vertex_as<rrt_node_t>(goal_vertex)->cost_to_come > new_tree_node->cost_to_come)
			{
				current_solution=new_tree_node->cost_to_come;
				current_solution_time = timer.measure();
				current_solution_iters = iteration_count;
				goal_vertex = node_index;
				//std::cout<<"Found new goal: "<<state_space->print_point(new_tree_node->point,3)<<" "<<new_tree_node->cost_to_come<<std::endl;
				std::cout <<"[" + planner_name + "] Found new goal:"<<state_space->print_point(new_tree_node->point,3);
				std::cout <<" cost:"<<new_tree_node->cost_to_come;
				std::cout<< " time:" << current_solution_time;
				std::cout<< " iter:" << current_solution_iters;
				std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;
				bnb(start_vertex,current_solution);
			}
		}
	}

	void rrt_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
	{
		auto node = tree.get_vertex_as<rrt_node_t>(v);
		const bool res = delete_flag || node->cost_to_come > cost_bound;
		std::list<node_index_t> children = node->get_children();
		for(auto child : children)
		{
			bnb(child,cost_bound,res);
		}
		children = node->get_children();
		if(res && children.empty())
		{
			//remove the node
			metric->remove_node(node.get());
			tree.remove_vertex(v);
		}
	}

	void rrt_t::print_statistics()
	{	
		std::cout <<"[" + planner_name + "]";
				std::cout<< " time:" <<  timer.measure();
				std::cout<< " iter:" << iteration_count;
				std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;
	}

}
