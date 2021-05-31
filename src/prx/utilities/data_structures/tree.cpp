
#include "prx/utilities/data_structures/tree.hpp"

namespace prx
{
	tree_t::tree_t()
	{
		vertex_count = 0;
		edge_count = 0;
		max_count = 0;
		vertex_id_counter = 0;
		edge_id_counter = 0;
	}
	tree_t::~tree_t()
	{
		
	}

	edge_index_t tree_t::add_edge(node_index_t from, node_index_t to)
	{
		prx_assert(v_index_map[to]->parent==to,"The node with index ["<<to<<"] already has a parent node ["<<v_index_map[to]<<"].");
		prx_assert(edge_count!=max_count,"There would now be more edges than vertices in the tree. This cannot happen.");

		v_index_map[from]->children.insert(v_index_map[from]->children.begin(),to);
		v_index_map[to]->parent = from;

		auto edge = *e_iter;
		edge->index = edge_id_counter;
		edge->source = from;
		edge->target = to;
		e_index_map[edge_id_counter]=edge;
		edge_id_counter++;
		e_iter++;
		const_e_iter++;
		edge_count++;
		v_index_map[to]->parent_edge = edge->index;

		return edge->index;
	}

	edge_index_t tree_t::add_safety_edge(node_index_t from, node_index_t to)
	{
		prx_assert(v_index_map[to]->parent==v_index_map[from]->parent,"The node with index ["<<from<<"] does not have the same parent as ["<<to<<"] instead: ["<<v_index_map[to]<<"].");
		prx_assert(edge_count!=max_count,"There would now be more edges than vertices in the tree. This cannot happen.");

		v_index_map[from]->children.insert(v_index_map[from]->children.begin(),to);
		v_index_map[to]->parent = from;

		auto edge = *e_iter;
		edge->index = edge_id_counter;
		edge->source = from;
		edge->target = to;
		e_index_map[edge_id_counter]=edge;
		edge_id_counter++;
		e_iter++;
		const_e_iter++;
		edge_count++;
		v_index_map[to]->parent_edge = edge->index;

		return edge->index;
	}

	unsigned tree_t::get_depth(node_index_t v)
	{
		if(v_index_map[v]->parent==v)
			return 0;
		unsigned depth = 1;
		while(v_index_map[v]->parent != v_index_map[v_index_map[v]->parent]->parent)
		{
			depth = depth+1;
			v = v_index_map[v]->parent;
		}
		return depth;
	}

	void tree_t::remove_vertex(node_index_t v)
	{
		prx_assert(v_index_map[v]->children.size()==0,"Can only remove a vertex if it doesn't have any children.");
		v_index_map[v_index_map[v]->parent]->children.remove(v);
		edge_index_t e = v_index_map[v]->parent_edge;

		auto temp_v = v_index_map[v];
		temp_v->parent=temp_v->index;
		v_index_map[v] = nullptr;
		auto v_iterator = std::find(vertex_list.begin(),v_iter,temp_v);
		v_iter--;
		const_v_iter--;
		*v_iterator = *v_iter;
		*v_iter = temp_v;
		vertex_count--;

		auto temp_e = e_index_map[e];
		auto e_iterator = std::find(edge_list.begin(),e_iter,temp_e);
		e_index_map[e] = nullptr;
		e_iter--;
		const_e_iter--;
		*e_iterator=*e_iter;
		*e_iter=temp_e;
		edge_count--;
	}

	void tree_t::purge()
	{
		clear();
		vertex_list.clear();
		edge_list.clear();
		v_index_map.clear();
		e_index_map.clear();
	}

	void tree_t::clear()
	{
		for(auto e : edge_list)
		{
			e->source=e->target = 0;
		}
		for(auto v : vertex_list)
		{
			v->children.clear();
			v->parent = v->index = 0;
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

	void tree_t::transplant(node_index_t root, node_index_t new_parent)
	{
		auto old_from = v_index_map[root]->parent;
		auto edge = e_index_map[v_index_map[root]->parent_edge];
		//remove root from its parent's child list. root is dangling
		v_index_map[old_from]->children.remove(root);
		//update parent index, need to update the parent edge though
		v_index_map[root]->parent = new_parent;
		//update parent's child list. still need to update edge
		v_index_map[new_parent]->children.insert(v_index_map[new_parent]->children.end(),root);
		edge->source = new_parent;
	}

}