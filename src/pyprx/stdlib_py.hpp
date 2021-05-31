#include <iostream>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

 
using namespace boost::python;
// typedef std::vector<double> double_list;
// std::ostream& operator<<(std::ostream&,std::vector<double>);

// template<typename T>
// void list_assign(std::list<T>& l, object o) {
//     // Turn a Python sequence into an STL input range
//     stl_input_iterator<T> begin(o), end;
//     l.assign(begin, end);
// }



template< typename T >
inline
std::vector< T > to_std_vector(boost::python::list& ns)
{
	std::vector< T > v;
    for (int i = 0; i < len(ns); ++i)
    {
        v.push_back(boost::python::extract<double>(ns[i]));
    }
    return v;
}

void pyprx_stdlib_py()
{
	
	class_<std::vector<double> >("std_vectorXd")
        .def(vector_indexing_suite<std::vector<double> >())
        .def(init<std::vector<double>>())
        .def("__len__", &std::vector<double>::size)
        // .def("__getitem__", &std_item<std::vector<double>::at, return_value_policy<copy_non_const_reference>())
        // .def("__setitem__", &std_item<std::vector<double>::at, with_custodian_and_ward<1,2>())
        // .def("assign", &list_assign<double>)
        .def("__str__", &to_str<double>) 
        .def("__repr__", &to_str<double>) 
        // .def(str(self))
        ;
        
    iterable_converter()
        // Build-in type.
        .from_python<std::vector<long unsigned> >()
        .from_python<std::vector<int> >()
        .from_python<std::vector<double> >()
        // Each dimension needs to be convertable.
        .from_python<std::vector<std::string> >()
        ;
}
