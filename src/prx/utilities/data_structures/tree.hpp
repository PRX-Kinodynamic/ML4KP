#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"
#include "prx/utilities/data_structures/abstract_edge.hpp"
#include "prx/utilities/spaces/space.hpp"

#include <unordered_map>
#include <list>

namespace prx
{
	class tree_t;

	/**
	 * @brief <b> A node on a tree. </b>
	 * 
	 * @author Zakary Littlefield
	*/
	class tree_node_t : public abstract_node_t
	{
	public:
		tree_node_t() : abstract_node_t(){}
		virtual ~tree_node_t(){}

		/**
		 * @brief Returns the index of the parent node.
		 * 
		 * @return Index of the parent node.
		*/
		node_index_t get_parent() const
		{
			return parent;
		}
		/**
		 * @brief Returns the index of the tree node.
		 * 
		 * @return Index of the node.
		*/
		node_index_t get_index() const
		{
			return index;
		}
		/**
		 * @brief Returns the index of the tree edge to this node.
		 * 
		 * @return Index of the edge from parent -> current node.
		*/
		edge_index_t get_parent_edge() const
		{
			return parent_edge;
		}

		/**
		 * @brief Returns the children of the current node.
		 * 
		 * @return A list containing the node indices of the child nodes.
		*/
		const std::list<node_index_t>& get_children() const
		{
			return children;
		}
	
	protected:

		node_index_t parent;
		edge_index_t parent_edge;
		node_index_t index;

		std::list<node_index_t> children;

		friend class tree_t;
	};

	/**
	 * @brief <b> An edge of a tree. </b>
	 * 
	 * @author Zakary Littlefield
	*/
	class tree_edge_t : public abstract_edge_t
	{
	public:
		tree_edge_t(){}
		
		virtual ~tree_edge_t(){}
		
		/**
		 * @brief Returns the index of the tree edge.
		 * 
		 * @return Index of the tree edge.
		*/
		edge_index_t get_index() const
		{
			return index;
		}

		/**
		 * @brief Returns the index of the source node.
		 * 
		 * @return Index of the parent or source node of this edge.
		*/
		node_index_t get_source() const
		{
			return source;
		}

		/**
		 * @brief Returns the index of the target node.
		 * 
		 * @return Index of the child or target node of this edge.
		*/
		node_index_t get_target() const
		{
			return target;
		}

	protected:
		node_index_t source;
		node_index_t target;

		edge_index_t index;
		friend class tree_t;
	};

	/**
	 * 
	 * @brief <b> A tree data structure. </b>
	 * 
	 * @author Zakary Littlefield
	*/
	class tree_t
	{
	public:
		typedef std::list<std::shared_ptr<tree_node_t>>::const_iterator const_vertex_iterator;
		typedef std::list<std::shared_ptr<tree_edge_t>>::const_iterator const_edge_iterator;
		typedef std::list<std::shared_ptr<tree_node_t>>::iterator vertex_iterator;
		typedef std::list<std::shared_ptr<tree_edge_t>>::iterator edge_iterator;

		template<class node_type,class edge_type>
		void allocate_memory(unsigned new_size)
		{
			unsigned old_size = vertex_count;
			if(max_count < new_size)
			{
				v_index_map.resize(new_size);
				e_index_map.resize(new_size);
				for(unsigned i=max_count; i<new_size; i++)
				{
					vertex_list.insert(vertex_list.end(),std::make_shared<node_type>());
					edge_list.insert(edge_list.end(),std::make_shared<edge_type>());
				}
			}
			if(old_size==0)
			{
				v_iter = vertex_list.begin();
				e_iter = edge_list.begin();
				const_v_iter = vertex_list.begin();
				const_e_iter = edge_list.begin();
			}
			else
			{
				v_iter = vertex_list.begin();
				e_iter = edge_list.begin();
				const_v_iter = vertex_list.begin();
				const_e_iter = edge_list.begin();
				std::advance(v_iter,old_size);
				std::advance(const_v_iter,old_size);
				std::advance(e_iter,old_size-1);
				std::advance(const_e_iter,old_size-1);
			}
			max_count = new_size;
		}

		template<class node_type,class edge_type>
		node_index_t add_vertex()
		{
			if(vertex_id_counter==max_count)
			{
				allocate_memory<node_type,edge_type>(vertex_list.size()*2+1);
			}
			auto node = *v_iter;
			node->index = vertex_id_counter;
			node->parent = vertex_id_counter;
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
		std::shared_ptr<edge_type> get_edge_as(edge_index_t e) const
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
		std::shared_ptr<tree_node_t> operator[](node_index_t v) const
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

		bool is_leaf(node_index_t v)
		{
			return (v_index_map[v]->get_children().size()==0);
		}

		edge_index_t edge(node_index_t from,node_index_t to) const
		{
			return v_index_map[to]->parent_edge;
		}

		tree_t();
		~tree_t();

		edge_index_t add_edge(node_index_t from, node_index_t to);
		edge_index_t add_safety_edge(node_index_t from, node_index_t to);

		unsigned get_depth(node_index_t v);

		void remove_vertex(node_index_t v);

		void purge();

		void clear();

		void transplant(node_index_t root, node_index_t new_parent);
		unsigned vertex_id_counter;

	protected:

		std::list<std::shared_ptr<tree_node_t>> vertex_list;
		std::list<std::shared_ptr<tree_edge_t>> edge_list;
		vertex_iterator v_iter;
		edge_iterator e_iter;
		const_vertex_iterator const_v_iter;
		const_edge_iterator const_e_iter;
		unsigned vertex_count;
		unsigned edge_count;
		unsigned max_count;
		unsigned edge_id_counter;

		std::vector<std::shared_ptr<tree_node_t>> v_index_map;
		std::vector<std::shared_ptr<tree_edge_t>> e_index_map;

	};
}
