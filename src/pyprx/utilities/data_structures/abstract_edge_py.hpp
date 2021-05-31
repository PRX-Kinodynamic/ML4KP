#include "prx/utilities/data_structures/abstract_edge.hpp"

// PRX_SETTER(abstract_node_t, point)
// PRX_GETTER(abstract_node_t, point)

void pyprx_utilities_data_structures_abstract_edge_py()
{
	// class_<prx::node_index_t>("node_index", no_init);
	// class_<prx::abstract_node_t, std::shared_ptr<prx::abstract_node_t>, bases<prx::proximity_node_t>>("abstract_node", init<>())
	class_<prx::abstract_edge_t, std::shared_ptr<prx::abstract_edge_t>>("abstract_edge", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::abstract_edge_t>, default_call_policies()))
		;
}