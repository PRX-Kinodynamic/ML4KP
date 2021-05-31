#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include "prx/planning/world_model.hpp"
#include "prx/simulation/system_group.hpp"

using namespace boost::python;

// template<class T>
// py::list std_vector_to_py_list(const std::vector<T>& v)
// {
//     py::object get_iter = py::iterator<std::vector<T> >();
//     py::object iter = get_iter(v);
//     py::list l(iter);
//     return l;
// }

// template< typename T >
// inline
// std::vector< T > to_std_vector( const py::object& iterable )
// {
//     return std::vector< T >( py::stl_input_iterator< T >( iterable ),
//                              py::stl_input_iterator< T >( ) );
// }

void pyprx_planning_world_model_py()
{
	
	// class_<std::vector<std::shared_ptr<prx::movable_object_t>>>("vector_movable_object")
 //   		.def(vector_indexing_suite<std::vector<std::shared_ptr<prx::movable_object_t>>>())
	// 	;


	class_<std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>> >("context")
    	.def_readwrite("system_group",  &std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>::first)
    	.def_readwrite("collision_group", &std::pair<std::shared_ptr<prx::system_group_t>, std::shared_ptr<prx::collision_group_t>>::second)
    	;
		// void create_context(const std::string& context_name,const std::vector<std::string>& system_names,const std::vector<std::string>& obstacle_names);
		// std::pair<,std::shared_ptr<collision_group_t>> get_context(const std::string& context_name);
			
   	class_<prx::world_model_t<>>("world_model", init<std::vector<prx::system_ptr_t>, std::vector<std::shared_ptr<prx::movable_object_t>>>())
   	    .def("create_context", &prx::world_model_t<>::create_context)
   	    .def("get_context",    &prx::world_model_t<>::get_context)
   	    ;
   	
}