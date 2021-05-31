// Lets put python translating functions globally accesible here
// 
// 
// 
using namespace boost::python;

#define PRX_GETTER(obj, var) template<class T>  T   get_##obj##_##var(prx::obj& o){return o.var;}
#define PRX_SETTER(obj, var) template<class T> void set_##obj##_##var(prx::obj& o, T val){o.var = val;}

/*
 * Safety checking functions
 * 
 */
#define INDEX_CHECK(i, MAX, MSG )if(i<0 || i >= MAX) {PyErr_SetString(PyExc_IndexError, ("[ PRX::" + std::string(MSG) + " ] Index '" + std::to_string(i) + "' Out Of Range: [0, " + std::to_string(MAX) + ")").c_str());throw_error_already_set();}

/*
 * Function wrappers to interface python with raw/smart ptrs
 */
#define PRX_FUNC_WRAPPER(OBJ, PTR, FUNC) void OBJ##_##FUNC##_wrapper(prx::OBJ& o, std::shared_ptr<prx::PTR> smart_p) { o.FUNC(smart_p.get()); }


/*
 * Iterating functions    
 */

template<typename T>
std::string to_str(const std::vector<T> &v) 
{
   // using namespace std;
   std::ostringstream os;
   copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
   return os.str();
}

/*
 * Template functions to create smart pointers
 */
template<class T, class R>
std::shared_ptr<R> create_ptr(T* obj)
{
	return std::shared_ptr<R>( obj );
	// std::shared_ptr<R> new_ptr;
	// new_ptr.reset(&new_obstacle);
	// return new_ptr;
}

template<class T>
std::shared_ptr<T> create_system_ptr(std::string name)
{

	return std::make_shared<T>( name );
}

template<class T, typename... Targs>
std::shared_ptr<T> init_as_ptr(Targs... Fargs)
{
  return std::make_shared<T>( Fargs... );
}

/*
 * High order functions!
 * Lipsy functions: take functions, create functions and return functions
 */
template<typename T, typename R, typename... Targs>
T create_function(PyObject* x)
{
    T new_f = [x](Targs... Fargs)
    {
        return call<R>(x, Fargs...);
    };
    return new_f;
}

/* 
 * String-related functions 
 */
template<typename T>
std::string prx_to_str(T obj)
{
    std::ostringstream iss;
    iss << obj;
  return iss.str();
} 

template<typename T>
void prx_print(T obj)
{
    std::cout << obj;
} 

/// @brief Type that allows for registration of conversions from
///        python iterable types.
struct iterable_converter
{
  /// @note Registers converter from a python interable type to the
  ///       provided type.
  template <typename Container>
  iterable_converter&
  from_python()
  {
    boost::python::converter::registry::push_back(
      &iterable_converter::convertible,
      &iterable_converter::construct<Container>,
      boost::python::type_id<Container>());

    // Support chaining.
    return *this;
  }

  /// @brief Check if PyObject is iterable.
  static void* convertible(PyObject* object)
  {
    return PyObject_GetIter(object) ? object : NULL;
  }

  /// @brief Convert iterable PyObject to C++ container type.
  ///
  /// Container Concept requirements:
  ///
  ///   * Container::value_type is CopyConstructable.
  ///   * Container can be constructed and populated with two iterators.
  ///     I.e. Container(begin, end)
  template <typename Container>
  static void construct(
    PyObject* object,
    boost::python::converter::rvalue_from_python_stage1_data* data)
  {
    namespace python = boost::python;
    // Object is a borrowed reference, so create a handle indicting it is
    // borrowed for proper reference counting.
    python::handle<> handle(python::borrowed(object));

    // Obtain a handle to the memory block that the converter has allocated
    // for the C++ type.
    typedef python::converter::rvalue_from_python_storage<Container>
                                                                storage_type;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    typedef python::stl_input_iterator<typename Container::value_type>
                                                                    iterator;

    // Allocate the C++ type into the converter's memory block, and assign
    // its handle to the converter's convertible variable.  The C++
    // container is populated by passing the begin and end iterators of
    // the python object to the container's constructor.
    new (storage) Container(
      iterator(python::object(handle)), // begin
      iterator());                      // end
    data->convertible = storage;
  }
};