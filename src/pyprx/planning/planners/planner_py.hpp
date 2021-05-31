#include <iostream>
#include <boost/python.hpp>
#include "prx/planning/planners/planner.hpp"

using namespace boost::python;


PRX_SETTER(planner_query_t, solution_traj)
PRX_GETTER(planner_query_t, solution_traj)

PRX_SETTER(planner_query_t, solution_plan)
PRX_GETTER(planner_query_t, solution_plan)

PRX_SETTER(planner_query_t, solution_cost)
PRX_GETTER(planner_query_t, solution_cost)

PRX_SETTER(planner_query_t, tree_visualization)
PRX_GETTER(planner_query_t, tree_visualization)

PRX_SETTER(planner_query_t, start_state)
PRX_GETTER(planner_query_t, start_state)

PRX_SETTER(planner_query_t, goal_state)
PRX_GETTER(planner_query_t, goal_state)

PRX_SETTER(planner_query_t, goal_region_radius)
PRX_GETTER(planner_query_t, goal_region_radius)

PRX_SETTER(planner_query_t, get_visualization)
PRX_GETTER(planner_query_t, get_visualization)

struct planner_wrapper : prx::planner_t, wrapper<prx::planner_t>
{
	planner_wrapper(const std::string& new_name) : prx::planner_t(new_name){};

	virtual void _link_and_setup_spec(prx::planner_specification_t* spec) override
    {
        this -> get_override("_link_and_setup_spec")();
        // this -> _link_and_setup_spec();
    }
	// virtual void _link_and_setup_spec(prx::planner_specification_t* spec) override
		// {std::cout << "\"_link_and_setup_spec\" is virtual! Needs to be overriden" << std::endl;};
	virtual bool _preprocess() override
	{
        return this -> get_override("_preprocess")();
	}
		// {std::cout << "\"_preprocess\" is virtual! Needs to be overriden" << std::endl;
		//  return false;};
	virtual bool _link_and_setup_query(prx::planner_query_t* query) override
	{
		return this -> get_override("_link_and_setup_query");
	}
		// {std::cout << "\"_link_and_setup_query\" is virtual! Needs to be overriden" << std::endl;
		 // return false;};
	virtual void _resolve_query(prx::condition_check_t* condition) override
	{
        this -> get_override("_resolve_query")();
	}
		// {std::cout << "\"_resolve_query\" is virtual! Needs to be overriden" << std::endl;};
	virtual void _fulfill_query() override
	{
        this -> get_override("_fulfill_query")();
	}
		// {std::cout << "\"_fulfill_query\" is virtual! Needs to be overriden" << std::endl;};
	virtual void _reset() override
	{
        this -> get_override("_reset")();
	}
		// {std::cout << "\"_reset\" is virtual! Needs to be overriden" << std::endl;};

};

void pyprx_planning_planners_planner_py()
{

	class_<prx::planner_query_t, std::shared_ptr<prx::planner_query_t>>("planner_query", no_init)
		.def("__init__", make_constructor(&init_as_ptr<prx::planner_query_t,prx::space_t*, prx::space_t*>, default_call_policies(),(arg("state_space"), arg("control_space"))))
		.add_property("start_state", &get_planner_query_t_start_state<prx::space_point_t>, &set_planner_query_t_start_state<prx::space_point_t>)
		.add_property("goal_state", &get_planner_query_t_goal_state<prx::space_point_t>, &set_planner_query_t_goal_state<prx::space_point_t>)
		.add_property("goal_region_radius", &get_planner_query_t_goal_region_radius<double>, &set_planner_query_t_goal_region_radius<double>)
		.add_property("get_visualization", &get_planner_query_t_get_visualization<bool>, &set_planner_query_t_get_visualization<bool>)
		.add_property("solution_traj", &get_planner_query_t_solution_traj<prx::trajectory_t>, &set_planner_query_t_solution_traj<prx::trajectory_t>)
		.add_property("solution_plan", &get_planner_query_t_solution_plan<prx::plan_t>, &set_planner_query_t_solution_plan<prx::plan_t>)
		.add_property("solution_cost", &get_planner_query_t_solution_cost<double>, &set_planner_query_t_solution_cost<double>)
		.add_property("tree_visualization", &get_planner_query_t_tree_visualization<std::vector<prx::trajectory_t>>, &set_planner_query_t_tree_visualization<std::vector<prx::trajectory_t>>)
		;
	class_<prx::planner_specification_t, std::shared_ptr<prx::planner_specification_t>, boost::noncopyable>("planner_specification", init<>())
		;
	class_<planner_wrapper, boost::noncopyable>("planner", no_init)
	// class_<std::shared_ptr<planner_wrapper>>("planner", no_init)
		.def("__init__", make_constructor(&init_as_ptr<planner_wrapper, std::string>, default_call_policies(),(arg("new_name"))))
		.def("link_and_setup_spec", &prx::planner_t::link_and_setup_spec )
		.def("preprocess", &prx::planner_t::preprocess )
		.def("link_and_setup_query", &prx::planner_t::link_and_setup_query )
		.def("resolve_query", &prx::planner_t::resolve_query )
		.def("fulfill_query", &prx::planner_t::fulfill_query )
		.def("reset", &prx::planner_t::reset )
		.def("get_statistics", 			pure_virtual(&prx::planner_t::get_statistics ) )
		.def("_link_and_setup_spec",  	&planner_wrapper::_link_and_setup_spec )
		.def("_preprocess", 		  	&planner_wrapper::_preprocess )
		.def("_link_and_setup_query", 	&planner_wrapper::_link_and_setup_query )
		.def("_resolve_query", 			&planner_wrapper::_resolve_query )
		.def("_fulfill_query", 			&planner_wrapper::_fulfill_query )
		.def("_reset", 					&planner_wrapper::_reset )
		// .def("", &prx::planner_t:: )
		;
}