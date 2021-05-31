
#include "prx/utilities/data_structures/undirected_graph.hpp"

namespace prx
{
	undirected_graph_t::undirected_graph_t()
	{
		vertex_count = 0;
		edge_count = 0;
		max_count = 0;
		vertex_id_counter = 0;
		edge_id_counter = 0;
		max_edge_count = 0;
	}
	undirected_graph_t::~undirected_graph_t()
	{
		
	}

	edge_index_t undirected_graph_t::add_edge(node_index_t first, node_index_t second, double _value)
	{

		// v_index_map[first]->neighbors.insert(v_index_map[first]->neighbors.begin(), second);
		// v_index_map[second]->neighbors.insert(v_index_map[second]->neighbors.begin(), first);
		v_index_map[first]->neighbors.push_front(second);
		v_index_map[second]->neighbors.push_front(first);

		// if (e_index_map.size() == edge_id_counter - 1)
		if(edge_id_counter==max_edge_count)
		{
			allocate_edges(e_index_map.size() * 2);
		}

		auto edge = *e_iter;
		edge->index = edge_id_counter;
		edge -> first = first;
		edge -> second = second;
		edge -> value = _value;
		// edge->source = from;
		// edge->target = to;
		e_index_map[edge_id_counter]=edge;
		edge_id_counter++;
		e_iter++;
		const_e_iter++;
		edge_count++;
		// v_index_map[second]->parent_edge = edge->index;

		// v_index_map[first] ->  edges.insert(v_index_map[first] -> edges.begin(),  edge -> index);
		// v_index_map[second] -> edges.insert(v_index_map[second] -> edges.begin(), edge -> index);
		v_index_map[first]  -> edges.push_back(edge -> index);
		v_index_map[second] -> edges.push_back(edge -> index);
		// std::cout << "edge: " << edge -> index << " first: " << first << " second: " << second;
		// std::cout << std::endl;

		return edge->index;
	}

	void undirected_graph_t::remove_vertex(node_index_t v)
	{
		// prx_assert(v_index_map[v]->children.size()==0,"Can only remove a vertex if it doesn't have any children.");
		// v_index_map[v_index_map[v]->parent]->children.remove(v);
		for (auto e : v_index_map[v]->edges)
		{
			auto other_node = e_index_map[e] -> get_other(v);
			// remove v from its neighbor's list
			v_index_map[other_node] -> neighbors.remove(v);
			v_index_map[other_node] -> edges.remove(e);			
			
			// remove edges from internal structs
			auto temp_e = e_index_map[e];
			auto e_iterator = std::find(edge_list.begin(),e_iter,temp_e);
			e_index_map[e] = nullptr;
			e_iter--;
			const_e_iter--;
			*e_iterator=*e_iter;
			*e_iter=temp_e;
			edge_count--;
		}

		// remove v from internal structs
		auto temp_v = v_index_map[v];
		// temp_v->parent=temp_v->index;
		v_index_map[v] = nullptr;
		auto v_iterator = std::find(vertex_list.begin(),v_iter,temp_v);
		v_iter--;
		const_v_iter--;
		*v_iterator = *v_iter;
		*v_iter = temp_v;
		vertex_count--;

	}

	void undirected_graph_t::purge()
	{
		clear();
		vertex_list.clear();
		edge_list.clear();
		v_index_map.clear();
		e_index_map.clear();
	}

	void undirected_graph_t::clear()
	{
		for(auto e : edge_list)
		{
			e -> first = e -> second = 0;
		}
		for(auto v : vertex_list)
		{
			v -> neighbors.clear();
			v -> edges.clear();
			v -> index = 0;
		}
		vertex_count = 0;
		edge_count = 0;
		edge_id_counter = 0;
		vertex_id_counter = 0;
		max_count=0;
		v_iter = vertex_list.begin();
		e_iter = edge_list.begin();
		const_v_iter = vertex_list.begin();
		const_e_iter = edge_list.begin();
	}

	void undirected_graph_t::dijkstra(node_index_t goal)
	{
		prx_assert(v_index_map[goal] != nullptr, "Goal node: " << goal << " not in the graph!");
		
		auto cmp = [](ug_node_ptr a, ug_node_ptr b)
		{
			return !(a -> cost_to_go < b -> cost_to_go);
		};

		auto goal_node = v_index_map[goal];
		goal_node -> cost_to_go = 0.0;

		
		std::set<node_index_t> visited;
		std::priority_queue<ug_node_ptr, std::vector<ug_node_ptr>, decltype(cmp) > pq(cmp);

		pq.push(goal_node);
		int i = 0;
		while (!pq.empty())
		{
			auto u = pq.top();
			pq.pop();
			auto search = visited.find(u -> index);

			if (search != std::end(visited)) continue;

			visited.insert(u -> index);
			// std::cout << "n: " << u -> index << " " << u -> point << " cost: " << u -> cost_to_go << std::endl;
			auto cost = u -> cost_to_go;
			for (auto e : u -> edges)
			{
				auto edge = e_index_map[e];
				auto candidate = v_index_map[edge -> get_other(u -> index)];

				if ( cost + edge -> value <= candidate -> cost_to_go)
				{
					// for (int i = 0; i < candidate -> point -> get_dim(); ++i)
					// {
					// 	std::cout << candidate -> point -> at(i) << " ";
					// }
					// printf("Cost: %.2f\n", cost + edge -> value );
					candidate -> cost_to_go = cost + edge -> value;
					candidate -> best_neighbor = u -> index;
					// visited.erase(candidate -> index);
				}
				// std::cout << "\tc: " << candidate -> index << " " << candidate -> point << std::endl;
					pq.push(candidate);

			}
			// printf("%d - index: %lu\tcost_to_go: %.4f\t pt=(%.0f, %.0f)\n", 
				// i, u -> index, u -> cost_to_go, u -> point -> at(1), u -> point -> at(0));
			// i++;
		}

	}

	void undirected_graph_t::vertex_list_to_file(std::string file_name)
	{
		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		// auto v = vertex_list.begin();
		// while (v != const_v_iter)
		// {
		// 	if (*v == nullptr) continue; // sanity check
		// 	ofs_map << "Point: ";
		// 	for (int i = 0; i < (*v) -> point -> get_dim(); ++i)
		// 	{
		// 		ofs_map << (*v) -> point -> at(i) << " ";
		// 	}
		// 	auto bn = v_index_map[(*v) -> best_neighbor];
		// 	for (int i = 0; i < bn -> point -> get_dim(); ++i)
		// 	{
		// 		ofs_map << bn -> point -> at(i) << " ";
		// 	}
		// 	// printf("cost: %.2f\n", (*v) -> cost_to_go);
		// 	ofs_map << "cost: " << ((*v) -> cost_to_go == std::numeric_limits<double>::infinity() ? std::numeric_limits<double>::max() : (*v) -> cost_to_go);

		// 	ofs_map << std::endl;
		// 	std::advance(v, 1);
		// }
		// 
		// 
		for (auto v : v_index_map)
		{
			if (v == nullptr) continue; // sanity check
			// for (auto e : v -> edges)
			// {
				// auto edge = e_index_map[e];
				// auto candidate = v_index_map[edge -> get_other(v -> index)];
			ofs_map << "Point: ";
			for (int i = 0; i < v -> point -> get_dim(); ++i)
			{
				ofs_map << v -> point -> at(i) << " ";
			}
			auto candidate = v_index_map[ v -> best_neighbor];
			for (int i = 0; i < candidate -> point -> get_dim(); ++i)
			{
				ofs_map << candidate -> point -> at(i) << " ";
			}
			// ofs_map << "cost: " << edge -> value;
			ofs_map << "cost: " << (v -> cost_to_go == std::numeric_limits<double>::infinity() ? std::numeric_limits<double>::max() : v -> cost_to_go);
			ofs_map << std::endl;
			// }
		}

	}

}