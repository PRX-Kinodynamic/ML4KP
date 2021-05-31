#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>

#include "prx/utilities/general/transforms.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace boost::python;
typedef Eigen::Matrix<int,1,1>::Index Index;
typedef Eigen::Matrix<int,1,1>::Scalar Scalar; 
typedef Eigen::AngleAxis<double> AngleAxisT;
// enum{Dim=VectorT::RowsAtCompileTime};
// static bool dyn(){ return Dim==Eigen::Dynamic; }

static inline void IDX_CHECK(Index i, Index MAX)
    { if(i<0 || i>=MAX) 
        { PyErr_SetString(PyExc_IndexError,("Index " + boost::lexical_cast<std::string>(i)+" out of range 0.." + boost::lexical_cast<std::string>(MAX-1)).c_str()); throw_error_already_set(); } }
static inline void IDX2_CHECKED_TUPLE_INTS(tuple tuple,const Index max2[2], Index arr2[2]) 
    {Index l=len(tuple); if(l!=2) { PyErr_SetString(PyExc_IndexError,"Index must be integer or a 2-tuple"); throw_error_already_set(); } for(int _i=0; _i<2; _i++) { extract<Index> val(tuple[_i]); if(!val.check()){ PyErr_SetString(PyExc_ValueError,("Unable to convert "+boost::lexical_cast<std::string>(_i)+"-th index to integer.").c_str()); throw_error_already_set(); } Index v=val(); IDX_CHECK(v,max2[_i]); arr2[_i]=v; }  }


template<typename T>
static void set_vector_item(T m, Index ix, Scalar value)
    { IDX_CHECK(ix, (Index) m.size() ); m[ix]=value; }

template<typename T>
static void set_matrix_item(T m, Index ix, Index iy, Scalar value)
    { IDX_CHECK(ix, (Index) m.size() ); m(ix, iy)=value; }

template<typename T>
static Scalar get_item(const T& a, tuple _idx){ Index idx[2]; Index mx[2]={a.rows(),a.cols()}; IDX2_CHECKED_TUPLE_INTS(_idx,mx,idx); return a(idx[0],idx[1]); }
template<typename T>
static void set_item(T& a, tuple _idx, const Scalar& value){ Index idx[2]; Index mx[2]={a.rows(),a.cols()}; IDX2_CHECKED_TUPLE_INTS(_idx,mx,idx); a(idx[0],idx[1])=value; }

template<typename T>
static T transpose(const T& m){ return m.transpose(); }
template<typename T, typename R>
static R diagonal(const T& m){ return m.diagonal(); }

template<typename T>
static T __imul__(T& a, const T& b){ a*=b; return a; };
template<typename T>
static T __mul__(const T& a, const T& b){ return a*b; }
template<typename T, typename R>
static R __mul__vec(const T& m, const R& v){ return m*v; }
// float matrices only
template<typename T>
static T inverse(const T& m){ return m.inverse(); }
template<typename T>
static T __div__(const T& a, const T& b){ return a/b; }

template<typename T>
static T Ones(Index rows, Index cols){     return T::Ones(rows,cols); }
template<typename T>
static T Zero(Index rows, Index cols){     return T::Zero(rows,cols); }
template<typename T>
static T Random(Index rows, Index cols){   return T::Random(rows,cols); }
template<typename T>
static T Identity(Index rows, Index cols){ return T::Identity(rows,cols); }

static prx::quaternion_t* fromAxisAngle(const prx::vector_t& axis, const Scalar& angle){ prx::quaternion_t* ret=new prx::quaternion_t(AngleAxisT(angle, axis)); ret->normalize(); return ret; }
static prx::quaternion_t* fromAngleAxis(const Scalar& angle, const prx::vector_t& axis){ prx::quaternion_t* ret=new prx::quaternion_t(AngleAxisT(angle, axis)); ret->normalize(); return ret; }
static prx::quaternion_t* fromTwoVectors(const prx::vector_t& u, const prx::vector_t& v){ prx::quaternion_t* q(new prx::quaternion_t); q->setFromTwoVectors(u,v); return q; }


// static 
// static auto translation(Eigen::Transform<double, 3, Eigen::AffineCompact> Tr)
//     {return Tr.translation();}
static void translation(prx::transform_t& Tr, prx::vector_t v)
    {Tr.translation() = (v);}

void pyprx_utilities_general_transforms()
{

    // using n_vector_t = Eigen::Matrix<double, N, 1>;
    // template <int N>
    // using n_matrix_t = Eigen::Matrix<double, N, N>;
    // using vector_t = n_vector_t<3>;
    // using matrix_t = n_matrix_t<3>;
    // using quaternion_t = Eigen::Quaternion<double>;
    // using axis_angle_t = Eigen::AngleAxis<double>;

    class_< prx::n_vector_t<Eigen::Dynamic> >("n_vector", init< prx::n_vector_t<Eigen::Dynamic> >() )
        .def("__setitem__", &set_vector_item< prx::n_vector_t<Eigen::Dynamic> >)
        ;

    class_< prx::n_matrix_t<Eigen::Dynamic> >("n_matrix", init< prx::n_matrix_t<Eigen::Dynamic> >() )
        .def("__setitem__", &set_matrix_item< prx::n_matrix_t<Eigen::Dynamic> >)
        ;

    class_< prx::vector_t >("vector", init< prx::vector_t >() )
        .def(init<double, double, double>())
        .def("__setitem__", &set_vector_item< prx::vector_t >)
        ;

    class_< prx::matrix_t >("matrix", init< prx::matrix_t >() )
        .def("__setitem__", &set_matrix_item< prx::matrix_t >)
        .def("determinant",&prx::matrix_t::determinant,"Return matrix determinant.")
        .def("trace",&prx::matrix_t::trace,"Return sum of diagonal elements.")
        .def("transpose",&transpose<prx::matrix_t>,"Return transposed matrix.")
        .def("diagonal",&diagonal<prx::matrix_t, prx::vector_t>,"Return diagonal as vector.")
        // // matrix*matrix product
        .def("__mul__",&__mul__<prx::matrix_t>).def("__imul__",&__imul__<prx::matrix_t>)
        // // matrix*vector product
        .def("__mul__",&__mul__vec<prx::matrix_t, prx::vector_t>).def("__rmul__",&__mul__vec<prx::matrix_t, prx::vector_t>)
        .def("Zero",&Zero<prx::matrix_t>,(arg("rows"),arg("cols")),"Create zero matrix of given dimensions").staticmethod("Zero")
        .def("Ones",&Zero<prx::matrix_t>,(arg("rows"),arg("cols")),"Create matrix of given dimensions where all elements are set to 1.").staticmethod("Ones")
        .def("Random",&Zero<prx::matrix_t>,(arg("rows"),arg("cols")),"Create matrix with given dimensions where all elements are set to number between 0 and 1 (uniformly-distributed).").staticmethod("Random")
        .def("Identity",&Zero<prx::matrix_t>,(arg("rank")),"Create identity matrix with given rank (square).").staticmethod("Identity")
        // .def("__setitem__",&prx::matrix_t::set_row).def("__getitem__",&prx::matrix_t::get_row)
        // .def("__setitem__",&prx::matrix_t::set_item< prx::matrix_t >).def("__getitem__",&prx::matrix_t::get_item< prx::matrix_t >)
        // .def("__str__",&prx::matrix_t::__str__).def("__repr__",&prx::matrix_t::__str__)
        ;

    class_< prx::quaternion_t >("quaternion" )
        .def("__init__", make_constructor(&fromAxisAngle, default_call_policies(),(arg("axis"),  arg("angle"))))
        .def("__init__", make_constructor(&fromAngleAxis, default_call_policies(),(arg("angle"), arg("axis"))))
        .def("__init__", make_constructor(&fromTwoVectors,default_call_policies(),(arg("u"),     arg("v"))))
        .def(init<Scalar,Scalar,Scalar,Scalar>((arg("w"),arg("x"),arg("y"),arg("z")),"Initialize from coefficients.\n\n.. note:: The order of coefficients is *w*, *x*, *y*, *z*. The [] operator numbers them differently, 0...4 for *x* *y* *z* *w*!"))
        .def(init<prx::matrix_t>((arg("rotMatrix")))) //,"Initialize from given rotation matrix.")
        .def(init<prx::quaternion_t>((arg("other"))))
        ;

    class_<prx::transform_t >("transform")
        // .def("__init__", make_constructor(&fromAxisAngle, default_call_policies(),(arg("axis"),  arg("angle"))))
        .def("setIdentity", &prx::transform_t::setIdentity)
        .def("translation", &translation)
        ;



}
