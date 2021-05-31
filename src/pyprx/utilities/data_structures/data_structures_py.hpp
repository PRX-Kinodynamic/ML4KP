#include <iostream>
#include <boost/python.hpp>
#include "pyprx/utilities/data_structures/gnn_py.hpp"
#include "pyprx/utilities/data_structures/abstract_node_py.hpp"
#include "pyprx/utilities/data_structures/abstract_edge_py.hpp"
#include "pyprx/utilities/data_structures/tree_py.hpp"

void pyprx_utilities_data_structures_py()
{
	pyprx_utilities_data_structures_gnn_py();
	pyprx_utilities_data_structures_abstract_edge_py();
	pyprx_utilities_data_structures_abstract_node_py();
	pyprx_utilities_data_structures_tree_py();
}