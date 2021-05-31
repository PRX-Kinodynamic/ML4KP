#include "prx/planning/planners/sst.hpp"

PRX_SETTER(sst_node_t, bridge)
PRX_GETTER(sst_node_t, bridge)

PRX_SETTER(sst_node_t, witness_index)
PRX_GETTER(sst_node_t, witness_index)

PRX_SETTER(witness_node_t, rep_index)
PRX_GETTER(witness_node_t, rep_index)

PRX_SETTER(witness_node_t, set)
PRX_GETTER(witness_node_t, set)

PRX_SETTER(sst_specification_t, delta_near)
PRX_GETTER(sst_specification_t, delta_near)

PRX_SETTER(sst_specification_t, delta_drain)
PRX_GETTER(sst_specification_t, delta_drain)


PRX_FUNC_WRAPPER(sst_t, rrt_query_t, link_and_setup_query)
PRX_FUNC_WRAPPER(sst_t, condition_check_t, resolve_query)


void pyprx_planning_planners_sst_py()
{
	class_<prx::sst_node_t, bases<prx::rrt_node_t>>("sst_node", init<>())
		.add_property("bridge", &get_sst_node_t_bridge<bool>, &get_sst_node_t_bridge<bool>)
		.add_property("witness_index", &get_sst_node_t_witness_index<prx::node_index_t>, &get_sst_node_t_witness_index<prx::node_index_t>)
		;

	class_<prx::witness_node_t, bases<prx::tree_node_t>>("witness_node", init<>())
		.add_property("rep_index", &get_witness_node_t_rep_index<prx::node_index_t>, &get_witness_node_t_rep_index<prx::node_index_t>)
		.add_property("set", &get_witness_node_t_set<bool>, &get_witness_node_t_set<bool>)
		;

	class_<prx::sst_specification_t, std::shared_ptr<prx::sst_specification_t>, bases<prx::rrt_specification_t>>("sst_specification", init<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>())
		.add_property("delta_near", 		&get_sst_specification_t_delta_near<double>, 	&set_sst_specification_t_delta_near<double>)
		.add_property("delta_drain",		&get_sst_specification_t_delta_drain<double>, 	&set_sst_specification_t_delta_drain<double>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		;

	class_<prx::sst_query_t, std::shared_ptr<prx::sst_query_t>, bases<prx::rrt_query_t>>("sst_query", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::sst_query_t, prx::space_t*, prx::space_t*>, default_call_policies(),(arg("state_space"), arg("control_space"))))
		;

	class_<prx::sst_t, bases<prx::rrt_t> >("sst", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::sst_t, std::string>, default_call_policies(),(arg("new_name"))))
		.def("link_and_setup_spec",  &prx::sst_t::link_and_setup_spec)
		.def("preprocess", 			 &prx::sst_t::preprocess)
		.def("link_and_setup_query", &sst_t_link_and_setup_query_wrapper)
		.def("resolve_query", 		 &sst_t_resolve_query_wrapper)
		.def("fulfill_query", 		 &prx::sst_t::fulfill_query)
		;

	// class_<prx::hyb_aorrt2_stride_query_t, std::shared_ptr<prx::hyb_aorrt2_stride_query_t>>("hyb_aorrt2_stride_query", init<>())
}