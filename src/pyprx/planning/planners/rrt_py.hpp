#include "prx/planning/planners/rrt.hpp"


using namespace boost::python;

PRX_SETTER(rrt_specification_t, cost_function)
PRX_GETTER(rrt_specification_t, cost_function)

PRX_SETTER(rrt_specification_t, sample_state)
PRX_GETTER(rrt_specification_t, sample_state)

PRX_SETTER(rrt_specification_t, sample_plan)
PRX_GETTER(rrt_specification_t, sample_plan)

PRX_SETTER(rrt_specification_t, distance_function)
PRX_GETTER(rrt_specification_t, distance_function)

PRX_SETTER(rrt_specification_t, valid_check)
PRX_GETTER(rrt_specification_t, valid_check)

PRX_SETTER(rrt_specification_t, valid_stop_check)
PRX_GETTER(rrt_specification_t, valid_stop_check)

PRX_SETTER(rrt_specification_t, valid_state)
PRX_GETTER(rrt_specification_t, valid_state)

PRX_SETTER(rrt_specification_t, propagate)
PRX_GETTER(rrt_specification_t, propagate)

PRX_GETTER(rrt_specification_t, state_space)

PRX_GETTER(rrt_specification_t, control_space)

PRX_SETTER(rrt_specification_t, bnb)
PRX_GETTER(rrt_specification_t, bnb)

PRX_SETTER(rrt_specification_t, use_replanning)
PRX_GETTER(rrt_specification_t, use_replanning)

PRX_SETTER(rrt_specification_t, min_control_steps)
PRX_GETTER(rrt_specification_t, min_control_steps)

PRX_SETTER(rrt_specification_t, max_control_steps)
PRX_GETTER(rrt_specification_t, max_control_steps)

PRX_SETTER(rrt_node_t, cost_to_come)
PRX_GETTER(rrt_node_t, cost_to_come)


PRX_SETTER(rrt_edge_t, plan)
PRX_GETTER(rrt_edge_t, plan)

PRX_SETTER(rrt_edge_t, traj)
PRX_GETTER(rrt_edge_t, traj)

PRX_SETTER(rrt_edge_t, edge_cost)
PRX_GETTER(rrt_edge_t, edge_cost)

PRX_FUNC_WRAPPER(rrt_t, rrt_query_t, link_and_setup_query)
PRX_FUNC_WRAPPER(rrt_t, condition_check_t, resolve_query)


void pyprx_planning_planners_rrt_py()
{
	class_<prx::rrt_node_t, bases<prx::tree_node_t>>("rrt_node", init<>())
		.add_property("cost_to_come", &get_rrt_node_t_cost_to_come<double>, &get_rrt_node_t_cost_to_come<double>)
		;

	class_<prx::rrt_edge_t, bases<prx::tree_edge_t>>("rrt_edge", init<>())
		.add_property("plan", 		&get_rrt_edge_t_plan<std::shared_ptr<prx::plan_t>>, 		&set_rrt_edge_t_plan<std::shared_ptr<prx::plan_t>>)
		.add_property("traj", 		&get_rrt_edge_t_traj<std::shared_ptr<prx::trajectory_t>>, 	&set_rrt_edge_t_traj<std::shared_ptr<prx::trajectory_t>>)
		.add_property("edge_cost", 	&get_rrt_edge_t_edge_cost<double>, 							&set_rrt_edge_t_edge_cost<double>)
		;

	// class_<prx::rrt_specification_t, std::shared_ptr<prx::rrt_specification_t>, bases<prx::planner_specification_t>>("rrt_specification", init<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>())
	class_<prx::rrt_specification_t, std::shared_ptr<prx::rrt_specification_t>, bases<prx::planner_specification_t>>("rrt_specification", no_init)
		.def("__init__", make_constructor(	&init_as_ptr<prx::rrt_specification_t, std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>, default_call_policies(),(arg("sg"), arg("cg"))))
		.add_property("cost_function", 		&get_rrt_specification_t_cost_function<prx::cost_function_t>, 			&set_rrt_specification_t_cost_function<prx::cost_function_t>)
		.add_property("distance_function",	&get_rrt_specification_t_distance_function<prx::distance_function_t>, 	&set_rrt_specification_t_distance_function<prx::distance_function_t>)
		.add_property("sample_state", 		&get_rrt_specification_t_sample_state<prx::sample_state_t>, 			&set_rrt_specification_t_sample_state<prx::sample_state_t>)
		.add_property("sample_plan", 		&get_rrt_specification_t_sample_plan<prx::sample_plan_t>, 				&set_rrt_specification_t_sample_plan<prx::sample_plan_t>)
		.add_property("valid_check", 		&get_rrt_specification_t_valid_check<prx::valid_trajectory_t>, 			&set_rrt_specification_t_valid_check<prx::valid_trajectory_t>)
		.add_property("valid_stop_check", 	&get_rrt_specification_t_valid_stop_check<prx::valid_stop_t>, 			&set_rrt_specification_t_valid_stop_check<prx::valid_stop_t>)
		.add_property("valid_state", 		&get_rrt_specification_t_valid_state<prx::valid_state_t>, 				&set_rrt_specification_t_valid_state<prx::valid_state_t>)
		.add_property("propagate", 			&get_rrt_specification_t_propagate<prx::propagate_t>, 					&set_rrt_specification_t_propagate<prx::propagate_t>)
		.def("state_space", 				&get_rrt_specification_t_state_space<prx::space_t*>, return_internal_reference<>())
		.def("control_space", 				&get_rrt_specification_t_control_space<prx::space_t*>, return_internal_reference<>())
		.add_property("bnb", 			    &get_rrt_specification_t_bnb<bool>, 									&set_rrt_specification_t_bnb<bool>)
		.add_property("use_replanning",	    &get_rrt_specification_t_use_replanning<bool>, 							&set_rrt_specification_t_use_replanning<bool>)
		.add_property("min_control_steps", 	&get_rrt_specification_t_min_control_steps<int>,						&set_rrt_specification_t_min_control_steps<int>)
		.add_property("max_control_steps", 	&get_rrt_specification_t_max_control_steps<int>,						&set_rrt_specification_t_max_control_steps<int>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		// .add_property("", 			    &get_rrt_specification_t_<>, 									&set_rrt_specification_t_<>)
		;

	class_<prx::rrt_query_t, std::shared_ptr<prx::rrt_query_t>, bases<prx::planner_query_t>>("rrt_query", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::rrt_query_t, prx::space_t*, prx::space_t*>, default_call_policies(),(arg("state_space"), arg("control_space"))))
		;

	class_<prx::rrt_t, std::shared_ptr<prx::rrt_t>, bases<prx::planner_t> >("rrt", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::rrt_t, std::string>, default_call_policies(),(arg("new_name"))))
		.def("link_and_setup_spec",  &prx::rrt_t::link_and_setup_spec)
		.def("preprocess", 			 &prx::rrt_t::preprocess)
		.def("link_and_setup_query", &rrt_t_link_and_setup_query_wrapper)
		.def("resolve_query", 		 &rrt_t_resolve_query_wrapper)
		.def("fulfill_query", 		 &prx::rrt_t::fulfill_query)
		;

}