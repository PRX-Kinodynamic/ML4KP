#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/abstract_edge.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <list>
#include <queue>
#include <fstream>
#include <unordered_map>
// #include <experimental/optional>

// Using prx_opt to avoid name conflicts with other libraries
// Specifically, optional could have a conflict with torch
#ifdef __cpp_lib_optional
    #include <optional>
    namespace prx_opt = std;
#elif __cpp_lib_experimental_optional
    #include <experimental/optional>
    namespace prx_opt = std::experimental;
#else
    #include <boost/optional/optional.hpp>
    namespace prx_opt = boost;

#endif

namespace prx
{
	class undirected_graph_t;

	class undirected_node_t : public abstract_node_t
	{
	public:
		undirected_node_t() : abstract_node_t()
		{
			cost_to_go = std::numeric_limits<double>::infinity();
		}
		virtual ~undirected_node_t(){}

		node_index_t get_index() const
		{
			return index;
		}

		// returns list of node_index's connected directly to this node. NOT the same as GNN
		const std::list<node_index_t>& get_neighbors() const
		{
			return neighbors;
		}

		const std::list<edge_index_t>& get_edges() const
		{
			return edges;
		}
		
		const double get_cost_to_go() const 
		{
			return cost_to_go;
		}

		const bool is_neighbor(node_index_t other) const
		{
			for (auto node : neighbors)
			{
				if (node == other) return true;
			}
			return false;
		}

		/**
		 * Get the best neighbor according to dijkstra(). If dijkstra has not been run
		 * or this node was not visited, the best neighbor is itself (best_neigbor == index)
		 * @return index of the best neighbor according to dijkstra().
		 */
		const node_index_t get_best_neighbor()
		{
			return best_neighbor;
		}

	protected:

		node_index_t index;

		std::list<edge_index_t> edges;
		std::list<node_index_t> neighbors;

		double cost_to_go;
		node_index_t best_neighbor;

		friend class undirected_graph_t;
	};

	class undirected_edge_t : public abstract_edge_t
	{
	public:
		undirected_edge_t()
		{
			// cost_to_go = ;
			value = std::numeric_limits<double>::max();
		}
		
		virtual ~undirected_edge_t(){}
		
		edge_index_t get_index() const
		{
			return index;
		}

		std::pair<node_index_t,node_index_t> get_connected_nodes() const
		{
			return std::make_pair(first, second);
		}

		bool is_node_connected(node_index_t node) const 
		{
			return node == first || node == second;
		}

		node_index_t get_other(node_index_t node) const
		{
			return node == first ? second : first;
		}

		void set_value(double _value)
		{
			value = _value;
		}

	protected:

		node_index_t first;
		node_index_t second;

		edge_index_t index;

		double value;
		friend class undirected_graph_t;
	};

	typedef std::shared_ptr<undirected_node_t> ug_node_ptr;
	typedef std::shared_ptr<undirected_edge_t> ug_edge_ptr;


	class undirected_graph_t
	{
	public:
		typedef std::list<ug_node_ptr>::const_iterator const_vertex_iterator;
		typedef std::list<ug_edge_ptr>::const_iterator const_edge_iterator;
		typedef std::list<ug_node_ptr>::iterator vertex_iterator;
		typedef std::list<ug_edge_ptr>::iterator edge_iterator;

		template<class node_type,class edge_type>
		void allocate_memory(unsigned new_size)
		{
			if(max_count < new_size)
			{
				allocate_vertex<node_type, edge_type>(new_size);
			}
			if(max_edge_count < new_size)
			{
				allocate_edges(new_size);

			}
		}

		template<class node_type,class edge_type>
		void allocate_vertex(unsigned new_size)
		{
			unsigned old_size = vertex_count;
			if(max_count < new_size)
			{
				v_index_map.resize(new_size);
				for(unsigned i=max_count; i<new_size; i++)
				{
					vertex_list.insert(vertex_list.end(),std::make_shared<node_type>());
				}
			}
			if(old_size==0)
			{
				v_iter = vertex_list.begin();
				const_v_iter = vertex_list.begin();
			}
			else
			{
				v_iter = vertex_list.begin();
				const_v_iter = vertex_list.begin();
				std::advance(v_iter,old_size);
				std::advance(const_v_iter,old_size);
			}
			max_count = new_size;
		}

		void allocate_edges(unsigned new_size)
		{
			unsigned old_size = edge_count;
			if(max_edge_count < new_size)
			{
				e_index_map.resize(new_size);
				for(unsigned i=max_edge_count; i<new_size; i++)
				{
					edge_list.insert(edge_list.end(),std::make_shared<undirected_edge_t>());
				}
			}
			if(old_size==0)
			{
				e_iter = edge_list.begin();
				const_e_iter = edge_list.begin();
			}
			else
			{
				e_iter = edge_list.begin();
				const_e_iter = edge_list.begin();
				std::advance(e_iter,old_size-1);
				std::advance(const_e_iter,old_size-1);
			}
			max_edge_count = new_size;
		}

		template<class node_type,class edge_type>
		node_index_t add_vertex()
		{
			if(vertex_id_counter==max_count)
			{
				allocate_vertex<node_type,edge_type>(vertex_list.size()*2+1);
			}
			auto node = *v_iter;
			node->index = vertex_id_counter;
			node->best_neighbor = vertex_id_counter;
			// node->parent = vertex_id_counter;
			v_index_map[vertex_id_counter] = node;
			vertex_id_counter++;
			v_iter++;
			const_v_iter++;
			vertex_count++;
			return node->index;
		}

		template<class node_type>
		std::shared_ptr<node_type> get_vertex_as(node_index_t v) const
		{
			return std::dynamic_pointer_cast<node_type>(v_index_map[v]);
		}

		template<class edge_type>
		ug_edge_ptr get_edge_as(edge_index_t e) const
		{
			return std::dynamic_pointer_cast<edge_type>(e_index_map[e]);
		}

		std::pair<const_vertex_iterator,const_vertex_iterator> vertices() const
		{
			return std::make_pair(vertex_list.begin(),const_v_iter);
		}

		std::pair<const_edge_iterator,const_edge_iterator> edges() const
		{
			return std::make_pair(edge_list.begin(),const_e_iter);
		}
		ug_node_ptr operator[](node_index_t v) const
		{
			return v_index_map[v];
		}
		unsigned num_vertices() const
		{
			return vertex_count;
		}

		unsigned num_edges() const
		{
			return edge_count;
		}

		// TODO: TEST THIS!!!
		prx_opt::optional<edge_index_t> edge(node_index_t first, node_index_t second) const
		{	
			std::function<bool(edge_index_t)> aux_f = [first,this](edge_index_t candidate)
			{
				return this -> e_index_map[candidate] -> is_node_connected(first);
			};
			const auto edges = v_index_map[second] -> edges;
			auto res = std::find_if(std::begin(edges), std::end(edges), aux_f);
			return prx_opt::optional<edge_index_t>(*res) ;
		}

		undirected_graph_t();
		~undirected_graph_t();

		edge_index_t add_edge(node_index_t from, node_index_t to, double _value = std::numeric_limits<double>::max());

		void remove_vertex(node_index_t v);

		void purge();

		void clear();

		unsigned vertex_id_counter;

		void dijkstra(node_index_t goal);

		void vertex_list_to_file(std::string file_name);

	protected:

		std::list<ug_node_ptr> vertex_list;
		std::list<ug_edge_ptr> edge_list;
		
		vertex_iterator v_iter;
		edge_iterator e_iter;
		const_vertex_iterator const_v_iter;
		const_edge_iterator const_e_iter;
		unsigned vertex_count;
		unsigned edge_count;
		unsigned max_count;
		unsigned max_edge_count;
		unsigned edge_id_counter;

		std::vector<ug_node_ptr> v_index_map;
		std::vector<ug_edge_ptr> e_index_map;

	};
}
