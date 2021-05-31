#include "prx/utilities/data_structures/tree.hpp"

PRX_SETTER(tree_t, vertex_id_counter)
PRX_GETTER(tree_t, vertex_id_counter)

void pyprx_utilities_data_structures_tree_py()
{
	class_<prx::tree_node_t, std::shared_ptr<prx::tree_node_t>, bases<prx::abstract_node_t>>("tree_node", init<>())
		.def("get_parent", &prx::tree_node_t::get_parent)
		.def("get_index", &prx::tree_node_t::get_index)
		.def("get_parent_edge", &prx::tree_node_t::get_parent_edge)
		.def("get_children", &prx::tree_node_t::get_children, return_internal_reference<>())
		;

	class_<prx::tree_edge_t, std::shared_ptr<prx::tree_edge_t>, bases<prx::abstract_edge_t>, boost::noncopyable>("tree_edge", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::tree_edge_t>, default_call_policies()))
		.def("get_index", &prx::tree_edge_t::get_index)
		.def("get_source", &prx::tree_edge_t::get_source)
		.def("get_target", &prx::tree_edge_t::get_target)
		;

	class_<prx::tree_t, std::shared_ptr<prx::tree_t>>("tree", init<>())
		.def("allocate_memory", &prx::tree_t::allocate_memory<prx::tree_node_t, prx::tree_edge_t>)
		.def("add_vertex", &prx::tree_t::add_vertex<prx::tree_node_t, prx::tree_edge_t>)
		.def("get_vertex_as", &prx::tree_t::get_vertex_as<prx::tree_node_t>)
		.def("get_edge_as", &prx::tree_t::get_edge_as<prx::tree_edge_t>)
		.def("num_vertices", &prx::tree_t::num_vertices)
		.def("num_edges", &prx::tree_t::num_edges)
		.def("is_leaf", &prx::tree_t::is_leaf)
		.def("edge", &prx::tree_t::edge)
		.def("add_edge", &prx::tree_t::add_edge)
		.def("add_safety_edge", &prx::tree_t::add_safety_edge)
		.def("get_depth", &prx::tree_t::get_depth)
		.def("remove_vertex", &prx::tree_t::remove_vertex)
		.def("purge", &prx::tree_t::purge)
		.def("clear", &prx::tree_t::clear)
		.def("transplant", &prx::tree_t::transplant)
		.add_property("vertex_id_counter", &get_tree_t_vertex_id_counter<unsigned>, &set_tree_t_vertex_id_counter<unsigned>)
		// .def("", &prx::tree_t::)
		;
}