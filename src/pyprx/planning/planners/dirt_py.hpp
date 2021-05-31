#include "prx/planning/planners/dirt.hpp"

PRX_SETTER(dirt_node_t, cost_to_go)
PRX_GETTER(dirt_node_t, cost_to_go)

PRX_SETTER(dirt_node_t, bridge)
PRX_GETTER(dirt_node_t, bridge)

PRX_SETTER(dirt_node_t, dir_radius)
PRX_GETTER(dirt_node_t, dir_radius)

PRX_SETTER(dirt_node_t, blossom_number)
PRX_GETTER(dirt_node_t, blossom_number)

PRX_SETTER(dirt_node_t, is_blossom_expand_done)
PRX_GETTER(dirt_node_t, is_blossom_expand_done)

PRX_SETTER(dirt_node_t, random_expand)
PRX_GETTER(dirt_node_t, random_expand)

// PRX_SETTER(dirt_node_t, edge_generators)
// PRX_GETTER(dirt_node_t, edge_generators)

PRX_SETTER(dirt_node_t, indices)
PRX_GETTER(dirt_node_t, indices)

PRX_SETTER(dirt_node_t, checkpoint_time)
PRX_GETTER(dirt_node_t, checkpoint_time)

PRX_SETTER(dirt_node_t, is_safety_node)
PRX_GETTER(dirt_node_t, is_safety_node)


PRX_SETTER(dirt_specification_t, blossom_number)
PRX_GETTER(dirt_specification_t, blossom_number)

PRX_SETTER(dirt_specification_t, use_pruning)
PRX_GETTER(dirt_specification_t, use_pruning)

PRX_SETTER(dirt_specification_t, replanning_cycle)
PRX_GETTER(dirt_specification_t, replanning_cycle)

PRX_SETTER(dirt_specification_t, order)
PRX_GETTER(dirt_specification_t, order)

PRX_SETTER(dirt_specification_t, h)
PRX_GETTER(dirt_specification_t, h)

// PRX_SETTER(dirt_specification_t, expand)
// PRX_GETTER(dirt_specification_t, expand)

// PRX_SETTER(dirt_specification_t, obstacle_distance_function)
// PRX_GETTER(dirt_specification_t, obstacle_distance_function)

PRX_SETTER(dirt_t, random_edges_counter)
PRX_GETTER(dirt_t, random_edges_counter)

PRX_SETTER(dirt_t, blossom_edges_counter)
PRX_GETTER(dirt_t, blossom_edges_counter)

PRX_FUNC_WRAPPER(dirt_t, dirt_query_t, link_and_setup_query)
PRX_FUNC_WRAPPER(dirt_t, condition_check_t, resolve_query)


void pyprx_planning_planners_dirt_py()
{
	class_<prx::dirt_node_t, bases<prx::rrt_node_t>>("dirt_node", init<>())
		.add_property("cost_to_go", 			&get_dirt_node_t_cost_to_go<double>, 			&get_dirt_node_t_cost_to_go<double>)
		.add_property("bridge", 				&get_dirt_node_t_bridge<bool>, 					&get_dirt_node_t_bridge<bool>)
		.add_property("dir_radius", 			&get_dirt_node_t_dir_radius<double>, 			&get_dirt_node_t_dir_radius<double>)
		.add_property("bridge", 				&get_dirt_node_t_bridge<bool>, 					&get_dirt_node_t_bridge<bool>)
		.add_property("dir_radius", 			&get_dirt_node_t_dir_radius<double>,  			&get_dirt_node_t_dir_radius<double>)
		.add_property("blossom_number", 		&get_dirt_node_t_blossom_number<int>, 			&get_dirt_node_t_blossom_number<int>)
		.add_property("is_blossom_expand_done", &get_dirt_node_t_is_blossom_expand_done<bool>,  &get_dirt_node_t_is_blossom_expand_done<bool>)
		.add_property("random_expand", 			&get_dirt_node_t_random_expand<bool>, 			&get_dirt_node_t_random_expand<bool>)
		// TODO: Add "std::vector<std::pair<plan_t*,trajectory_t*>>" class
		// .add_property("edge_generators", &get_sst_node_t_edge_generators<std::vector<std::pair<plan_t*,trajectory_t*>>>, &get_sst_node_t_edge_generators<std::vector<std::pair<plan_t*,trajectory_t*>>>)
		.add_property("indices", 			&get_dirt_node_t_indices<std::vector<int>>, &get_dirt_node_t_indices<std::vector<int>>)
		.add_property("checkpoint_time", 	&get_dirt_node_t_checkpoint_time<double>, 	&get_dirt_node_t_checkpoint_time<double>)
		.add_property("is_safety_node", 	&get_dirt_node_t_is_safety_node<bool>, 		&get_dirt_node_t_is_safety_node<bool>)
		;

	class_<prx::dirt_specification_t, std::shared_ptr<prx::dirt_specification_t>, bases<prx::rrt_specification_t>>("dirt_specification", init<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>())
		.add_property("blossom_number", 				&get_dirt_specification_t_blossom_number<int>, 					&set_dirt_specification_t_blossom_number<int>)
		.add_property("use_pruning", 					&get_dirt_specification_t_use_pruning<bool>, 					&set_dirt_specification_t_use_pruning<bool>)
		.add_property("replanning_cycle", 				&get_dirt_specification_t_replanning_cycle<double>, 			&set_dirt_specification_t_replanning_cycle<double>)
		.add_property("order", 							&get_dirt_specification_t_order<int>, 							&set_dirt_specification_t_order<int>)
		.add_property("h", 								&get_dirt_specification_t_h<prx::heuristic_function_t>, 		&set_dirt_specification_t_h<prx::heuristic_function_t>)
		// .add_property("expand", 						&get_dirt_specification_t_expand<prx::expand_t>, 				&set_dirt_specification_t_expand<prx::expand_t>)
		// .add_property("obstacle_distance_function", 	&get_dirt_specification_t_<prx::obstacle_distance_function_t>, 	&set_dirt_specification_t_<prx::obstacle_distance_function_t>)
		// .add_property("", 	&get_sst_specification_t_<>, 	&set_sst_specification_t_<>)
		// .add_property("", 	&get_sst_specification_t_<>, 	&set_sst_specification_t_<>)
		;

	class_<prx::dirt_query_t, std::shared_ptr<prx::dirt_query_t>, bases<prx::rrt_query_t>>("dirt_query", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::dirt_query_t, prx::space_t*, prx::space_t*>, default_call_policies(),(arg("state_space"), arg("control_space"))))
		;

	class_<prx::dirt_t, bases<prx::rrt_t> >("dirt", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::dirt_t, std::string>, default_call_policies(),(arg("new_name"))))
		.def("link_and_setup_spec",  &prx::dirt_t::link_and_setup_spec)
		.def("preprocess", 			 &prx::dirt_t::preprocess)
		.def("link_and_setup_query", &dirt_t_link_and_setup_query_wrapper)
		.def("resolve_query", 		 &dirt_t_resolve_query_wrapper)
		.def("fulfill_query", 		 &prx::dirt_t::fulfill_query)
		// .def("",   	 &prx::dirt_t::)
		.add_property("random_edges_counter", 	&get_dirt_t_random_edges_counter<std::vector<long unsigned>>,  &set_dirt_t_random_edges_counter<std::vector<long unsigned>> )
		.add_property("blossom_edges_counter",	&get_dirt_t_blossom_edges_counter<std::vector<long unsigned>>, &set_dirt_t_blossom_edges_counter<std::vector<long unsigned>> )
		// .add_property("",   	 &prx::dirt_t::)
		// .add_property("",   	 &prx::dirt_t::)
		;

	// class_<prx::hyb_aorrt2_stride_query_t, std::shared_ptr<prx::hyb_aorrt2_stride_query_t>>("hyb_aorrt2_stride_query", init<>())
}