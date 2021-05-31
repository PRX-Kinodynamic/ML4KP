#include "prx/utilities/data_structures/gnn.hpp"

PRX_SETTER(proximity_node_t, added_index)
PRX_GETTER(proximity_node_t, added_index)

std::vector<long unsigned> get_neighbors_wrapper(std::shared_ptr<prx::proximity_node_t> pn, long unsigned nr_neigh)
{
	std::vector<long unsigned> v;
    long unsigned* neighbors = pn -> get_neighbors( &nr_neigh );
	for( int i=0; i<nr_neigh; i++ )
    {
       v.push_back(neighbors[i]);
    }
    return v;
}

void pyprx_utilities_data_structures_gnn_py()
{
	class_<prx::proximity_node_t, std::shared_ptr<prx::proximity_node_t>>("proximity_node", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::proximity_node_t>, default_call_policies()))
		.def("get_prox_index", &prx::proximity_node_t::get_prox_index)
		.def("set_index", &prx::proximity_node_t::set_index)
		// .def("get_neighbors", &prx::proximity_node_t::get_neighbors, return_internal_reference<>())
		.def("get_neighbors", get_neighbors_wrapper)
		.def("add_neighbor", &prx::proximity_node_t::add_neighbor)
		.def("delete_neighbor", &prx::proximity_node_t::delete_neighbor)
		.def("replace_neighbor", &prx::proximity_node_t::replace_neighbor)
		.def("remove_all_neighbors", &prx::proximity_node_t::remove_all_neighbors)
		.add_property("added_index", &get_proximity_node_t_added_index<long unsigned>, &set_proximity_node_t_added_index<long unsigned>)
		// .def("", &prx::proximity_node_t::)
		;

	class_<prx::graph_nearest_neighbors_t, std::shared_ptr<prx::graph_nearest_neighbors_t>>("graph_nearest_neighbors", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::graph_nearest_neighbors_t, prx::distance_function_t>, default_call_policies(), (arg("distance_function"))))
		.def("node_distance", &prx::graph_nearest_neighbors_t::node_distance)
		.def("add_node", &prx::graph_nearest_neighbors_t::add_node)
		.def("remove_node", &prx::graph_nearest_neighbors_t::remove_node)
		.def("clear", &prx::graph_nearest_neighbors_t::clear)
		.def("get_nr_nodes", &prx::graph_nearest_neighbors_t::get_nr_nodes)
		.def("single_query", &prx::graph_nearest_neighbors_t::single_query, return_internal_reference<>())
		.def("multi_query", &prx::graph_nearest_neighbors_t::multi_query)
		.def("radius_and_closest_query", &prx::graph_nearest_neighbors_t::radius_and_closest_query)
		// .def("", &prx::graph_nearest_neighbors_t::)
		// .def("", &prx::graph_nearest_neighbors_t::)
		// .add_property("added_index", &get_proximity_node_t_added_index<long unsigned>, &set_proximity_node_t_added_index<long unsigned>)
		// .def("", &prx::proximity_node_t::)
		;
}