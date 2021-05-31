#include "prx/planning/planners/aorrt.hpp"

namespace prx
{
	aorrt_t::aorrt_t(const std::string& new_name) : rrt_t(new_name)
	{
		metric = nullptr;
		c_new = 0;
		planner_name = new_name;
	}
	aorrt_t::~aorrt_t()
	{
		_reset();
	}
	void aorrt_t::_link_and_setup_spec(planner_specification_t* spec)
	{
		//reset is always called before this
		aorrt_spec = dynamic_cast<aorrt_specification_t*>(spec);
		prx_assert(aorrt_spec!=nullptr,"RRT_2 received an incorrect specification.");
		distance_function = aorrt_spec -> distance_function;
		cost_function = aorrt_spec -> cost_function;
		sample_state = aorrt_spec -> sample_state;
		sample_plan = aorrt_spec -> sample_plan;
		valid_check = aorrt_spec -> valid_check;
		valid_stop_check = aorrt_spec -> valid_stop_check;
		propagate = aorrt_spec -> propagate;
		use_replanning = aorrt_spec -> use_replanning;

		cost_state_space = aorrt_spec -> cost_state_space;
		X_state_space = aorrt_spec -> X_state_space;
		Y_state_space = aorrt_spec -> Y_state_space;
		control_space = aorrt_spec -> control_space;
		expand = aorrt_spec->expand;

		// Y_min_cost = aorrt_spec -> Y_min_cost;

		metric = new graph_nearest_neighbors_t(distance_function);

		// Sample pts
		X_sample_point = X_state_space -> make_point();
		cost_sample_point = cost_state_space -> make_point();
		Y_sample_point = Y_state_space -> make_point();

		// Auxiliary pts
		X_aux_pt = X_state_space -> make_point();
		Y_aux_pt = Y_state_space -> make_point();
		cost_aux_pt = cost_state_space -> make_point();
		
		// Min pt
		Y_min = Y_state_space -> make_point();

		Y_min_cost = aorrt_spec -> c_max;
		c_max = aorrt_spec -> c_max;
	}
	bool aorrt_t::_preprocess()
	{
		tree.allocate_memory<aorrt_node_t, aorrt_edge_t>(1000);
		return true;
	}
	bool aorrt_t::_link_and_setup_query(planner_query_t* query)
	{
		aorrt_query = dynamic_cast<aorrt_query_t*>(query);
		prx_assert(aorrt_query!=nullptr,"RRT_2 received an incorrect query type.");

		if(tree.num_vertices()==0 || 
			!X_state_space->equal_points(
				tree.get_vertex_as<aorrt_node_t>(start_vertex) -> point, aorrt_query -> start_state) )
		{
			//clear existing data structure
			metric -> clear();
			tree.clear();
			start_vertex = tree.add_vertex<aorrt_node_t, aorrt_edge_t>();
			goal_vertex = start_vertex;
			auto start_node = tree.get_vertex_as<aorrt_node_t>(start_vertex);
			start_node->point = Y_state_space -> make_point();

			Y_state_space -> point_union(aorrt_query -> start_state, 
				cost_state_space -> get_start_state(), start_node->point );

			start_node -> cost_to_come = 0;
			metric -> add_node(start_node.get());
		}
		timer.reset();
		iteration_count=0;
		current_solution_iters=0;
		current_solution_time=0;

		return true;
	}

	void aorrt_t::_resolve_query(condition_check_t* condition)
	{
		// cost_state_space -> set_bounds({0.0}, {aorrt_spec -> c_max});
		bool sol_found = false;
		w_x_bk = aorrt_spec -> w_x;
		w_c_bk = aorrt_spec -> w_c;

		aorrt_spec -> w_x = 1.0;
		aorrt_spec -> w_c = 0.0;

		do
		{
			//sample state
			sample_state(X_sample_point);

			cost_state_space -> sample(cost_sample_point);
			Y_state_space -> point_union(X_sample_point, cost_sample_point, Y_sample_point);

			// std::cout << "X_sample_point: " << X_sample_point << std::endl;
			// std::cout << "cost_sample_point: " << cost_sample_point << std::endl;
			// std::cout << "Y_sample_point: " << Y_sample_point << std::endl;
			//find closest
			auto closest_node = static_cast<aorrt_node_t*>(metric->single_query(Y_sample_point));
			Y_state_space -> split_point(closest_node -> point, X_aux_pt, cost_aux_pt);


			// sample_plan(plan);
			std::vector<plan_t*> plans;
			std::vector<trajectory_t*> trajs;
			expand(X_aux_pt, plans, trajs, aorrt_spec -> blossom_number, false);
			plan_t plan(*plans.front());
			trajectory_t traj(*trajs.front());

			//propagate
			// plan_t plan(control_space);
			// trajectory_t traj(X_state_space);
			// propagate(X_aux_pt,plan,traj);

			c_new = cost_aux_pt -> at(0) + cost_function(traj,plan);
			//collision check and bnb
			// printf("valid_check: %s\tcost_comp: %s\n", valid_check(traj)?"true":"false", (c_new <  Y_min_cost)?"true":"false");
			// printf("c_new = %.4f + %.4f = %.4f\n", cost_aux_pt -> at(0), cost_function(traj,plan), c_new);

			if( valid_check(traj) &&
				c_new <  Y_min_cost )
			{
				//add vertex
				auto node_index = tree.add_vertex<aorrt_node_t, aorrt_edge_t>();
				auto new_tree_node = tree.get_vertex_as<aorrt_node_t>(node_index);
				cost_state_space -> set_cost(cost_aux_pt, c_new);
				Y_state_space -> point_union(traj.back(), cost_aux_pt, Y_aux_pt );
				new_tree_node -> point = Y_state_space -> clone_point(Y_aux_pt);

				// add node to metric
				metric->add_node(new_tree_node.get());

				// add edge
				edge_index_t edge_index = tree.add_edge(closest_node->get_index(), node_index);
				auto new_edge = tree.get_edge_as<aorrt_edge_t>(edge_index);
				new_edge->plan = std::make_shared<plan_t>(plan);
				new_edge->traj = std::make_shared<trajectory_t>(traj);

				update_goal(node_index);

			}
			iteration_count++;
		}
		while(!condition->check());

	}

	void aorrt_t::_fulfill_query()
	{
		if(goal_vertex!=start_vertex && !use_replanning)
		{
			//backtrack to get the plan and trajectory
			aorrt_query->solution_cost = Y_min_cost;
			std::deque<node_index_t> node_indices;
			node_index_t current_index = goal_vertex;
			
			trajectory_costs.clear();
			aorrt_query -> solution_traj.clear();
			aorrt_query -> solution_plan.clear();

			while(current_index!=start_vertex)
			{
				node_indices.push_front(current_index);
				current_index = tree[current_index] -> get_parent();

			}

			aorrt_query -> solution_plan = *tree.get_edge_as<aorrt_edge_t>(tree[node_indices[0]] -> get_parent_edge()) -> plan;
			aorrt_query -> solution_traj = *tree.get_edge_as<aorrt_edge_t>(tree[node_indices[0]] -> get_parent_edge()) -> traj;

			trajectory_costs.push_back(tree.get_vertex_as<aorrt_node_t>(node_indices[0]) -> point);

			for(int i=1;i<node_indices.size();i++)
			{
				aorrt_query -> solution_traj.resize(aorrt_query -> solution_traj.size() - 1);
				aorrt_query -> solution_plan += *tree.get_edge_as<aorrt_edge_t>(tree[node_indices[i]] -> get_parent_edge()) -> plan;
				aorrt_query -> solution_traj += *tree.get_edge_as<aorrt_edge_t>(tree[node_indices[i]] -> get_parent_edge()) -> traj;

				trajectory_costs.push_back(tree.get_vertex_as<aorrt_node_t>(node_indices[i]) -> point);
			}
		}
		else
		{
			aorrt_query -> solution_cost = 0;
		}
		if (use_replanning)
		{
			std::cout << "REPLANNING NOT IMPLEMENTED" << std::endl;

		}
		if(aorrt_query -> get_visualization)
		{
	        	auto iter_bounds = tree.edges();
	        	for(auto iter = iter_bounds.first; iter!=iter_bounds.second; iter++)
	        	{
				aorrt_query -> tree_visualization.push_back(*tree.get_edge_as<aorrt_edge_t>((*iter) -> get_index()) -> traj);
			}
		}
	}

	std::vector<double> aorrt_t::get_statistics()
	{
		//time, iters, nodes, solution quality, first_time, first_iters, current_solution
		return {timer.measure(),
				static_cast<double>(iteration_count),
				static_cast<double>(metric->get_nr_nodes()),
				Y_min_cost,
				current_solution_time,
				static_cast<double>(current_solution_iters)
				};
	}

	void aorrt_t::_reset()
	{
		//clear the stuff
		tree.purge();
		if(metric!=nullptr)
		{
			delete metric;
			metric = nullptr;
		}

	}

	void aorrt_t::update_goal(node_index_t node_index)
	{	
		auto new_tree_node = tree.get_vertex_as<aorrt_node_t>(node_index);
		auto tree_edge = tree.get_edge_as<aorrt_edge_t>(new_tree_node -> get_parent_edge());

		space_point_t pt = tree_edge -> traj -> back();
		// if(distance_function(aorrt_query->goal_state, traj.back()) < aorrt_query->goal_region_radius
		if(aorrt_query->goal_check( pt ) && c_new < Y_min_cost)
		{
			// statics
			current_solution = c_new;
			current_solution_time = timer.measure();
			current_solution_iters = iteration_count;

			// Update the solution
			Y_state_space -> copy_point(Y_min, Y_aux_pt);
			Y_min_cost = c_new;
			goal_vertex = node_index;

			Y_min_cost = c_new;
			aorrt_spec -> c_max = c_new;
			aorrt_spec -> w_x = w_x_bk;
			aorrt_spec -> w_c = w_c_bk;

			std::cout <<"[" + planner_name + "] Found new goal:( "<< pt << ")";
			std::cout <<" cost:"<< current_solution;
			std::cout<< " time:" << current_solution_time;
			std::cout<< " iter:" << current_solution_iters;
			std::cout<< " nodes:" << metric->get_nr_nodes() <<std::endl;

			cost_state_space -> set_bounds({0.0}, {aorrt_spec -> c_max});

			bnb(start_vertex, c_new);
		}
	}

	void aorrt_t::bnb(node_index_t v, double cost_bound, bool delete_flag)
	{
		auto node = tree.get_vertex_as<aorrt_node_t>(v);
		Y_state_space -> split_point(node -> point, X_aux_pt, cost_aux_pt);
		const double node_cost = cost_aux_pt -> at (0);
		const bool res = delete_flag || ( node_cost > cost_bound );
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

}
