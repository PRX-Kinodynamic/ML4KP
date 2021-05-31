#include "prx/utilities/data_structures/abstract_node.hpp"

PRX_SETTER(abstract_node_t, point)
PRX_GETTER(abstract_node_t, point)

void pyprx_utilities_data_structures_abstract_node_py()
{
	// class_<prx::node_index_t>("node_index", no_init);
	// class_<prx::abstract_node_t, std::shared_ptr<prx::abstract_node_t>, bases<prx::proximity_node_t>>("abstract_node", init<>())
	class_<prx::abstract_node_t, std::shared_ptr<prx::abstract_node_t>>("abstract_node", init<>())
		.add_property("point", &get_abstract_node_t_point<prx::space_point_t>, &set_abstract_node_t_point<prx::space_point_t>)
		;
}