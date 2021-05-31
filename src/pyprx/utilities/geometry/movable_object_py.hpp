#include <iostream>
#include <boost/python.hpp>
#include "prx/utilities/geometry/movable_object.hpp"

using namespace boost::python;

// template<class T>
// std::shared_ptr<prx::movable_object_t> create_obstacle_py(T new_obstacle)
// {
// 	std::shared_ptr<prx::movable_object_t> new_ptr;
// 	new_ptr.reset(&new_obstacle);
// 	return new_ptr;
// }

// struct movable_object_wrapper: prx::movable_object_t
// {
//     movable_object_wrapper(PyObject* self_, std::string p): prx::movable_object_t(p), self(self_) {}
//     // int f() { return call_method<int>(self, "f"); }    
//     // int default_movable_object_wrapper() { return movable_object_wrapper::f(); }    
//     PyObject* self;
// };


void pyprx_utilities_geometry_movable_object_py()
{

   	// class_<prx::movable_object_t, boost::noncopyable>("movable_object", init<std::string>())
   	// class_<prx::movable_object_t, movable_object_wrapper>("movable_object", init<PyObject*, std::string>())
   	    // ;
   	// class_<prx::movable_object_t, std::shared_ptr<prx::movable_object_t>, boost::noncopyable >("movable_object", init<std::string>())
   	class_<prx::movable_object_t, std::shared_ptr<prx::movable_object_t>, boost::noncopyable >("movable_object", init<std::string>())
   		// .def("create_obstacle", &create_ptr<prx::movable_object_t, prx::movable_object_t>).staticmethod("create_obstacle")
   		// .def("create_obstacle", &create_ptr<prx::movable_object_t, prx::movable_object_t>)
    	;

	iterable_converter()
        .from_python<std::vector<std::shared_ptr<prx::movable_object_t>>>()
   	    ;
    	
   	// def("create_obstacle", &create_ptr<prx::movable_object_t, prx::movable_object_t>, with_custodian_and_ward_postcall<0,1>());
	// register_ptr_to_python< std::shared_ptr<prx::movable_object_t> >();
    // def("create_obstacle", &create_ptr<prx::movable_object_t, prx::movable_object_t>);
}