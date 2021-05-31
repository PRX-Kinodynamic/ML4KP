#pragma once
#include <tuple>

#include "prx/utilities/defs.hpp"

namespace prx
{	
	// Forward declarations
	template<typename... Tn> 
    class zipped_t;

	// Forward declarations
	template<typename... Tp> 
	zipped_t<typename Tp::iterator...> zip_iters(Tp&... t);

	// Forward declarations
	template<typename... It>
	std::tuple<typename std::iterator_traits<It>::value_type...> unzip(std::tuple<It...>& t);

	// To use this class/functions:
	// for (auto t : prx::zip_iters(vec_of_integers, vec_of_strings, vec_of_doubles) )
	// {
	// 		std::tie(some_int, some_str, some_dbl) = prx::unzip(t);
	// 		(...)
	// }

    /**
    * @brief <b> A class to iterate over two or more iterable structures simultaneously </b>
    * 
    * A class to iterate over two or more iterable structures simultaneously
    * 
    * @author Edgar Granados
    */
    template<typename... Tn> 
    class zipped_t
    {
    	/**
    	 * iterator of the zipped_t class
    	 */
    	struct zipped_iterator 
		{
			using iterator_category = std::forward_iterator_tag;
			using difference_type   = std::ptrdiff_t;
			using value_type        = std::tuple<Tn...>;
			using pointer           = value_type*;  
			using reference         = value_type&;  

			reference operator*() const { return *m_ptr; }
    		pointer operator->() { return m_ptr; }

    		zipped_iterator(pointer t)
    		{
    			m_ptr = t;
    		}

    		template<std::size_t I = 0, typename... Tp>
			inline typename std::enable_if<I == sizeof...(Tp), void>::type
  			tuple_increment(std::tuple<Tp...>& t, int n)
  			{ 
  			}

    		template<std::size_t I = 0, typename... Tp>
    		inline typename std::enable_if < I < sizeof...(Tp), void>::type // >
    		tuple_increment(std::tuple<Tp...>& t, int n)
    		{
    			std::get<I>(t) += n;
    			tuple_increment<I + 1, Tp...>(t, n);
    		}

    		template<std::size_t I = 0, typename... Tp>
			inline typename std::enable_if<I == sizeof...(Tp), bool>::type
  			tuple_equal(const std::tuple<Tp...>& a, const std::tuple<Tp...>& b) const
  			{ 
  				return true;
  			}

  			template<std::size_t I = 0, typename... Tp>
    		inline typename std::enable_if < I < sizeof...(Tp), bool>::type // >
    		tuple_equal(const std::tuple<Tp...>& a, const std::tuple<Tp...>& b) const
    		{
    			if ( std::get<I>(a) == std::get<I>(b) )
    			{
    				return tuple_equal<I + 1, Tp...>(a, b);
    			}
    			return false;
    		}

    		// Prefix increment
    		zipped_iterator operator+(int b) 
    		{	
    			pointer tmp = new value_type(*m_ptr);
    			// std::cout << "val this before:" << *(std::get<0>(*(this -> m_ptr))) << std::endl;
    			// std::cout << "val before:" << *(std::get<0>(*(tmp))) << std::endl;
    			tuple_increment(*(tmp ), b);
    			// std::cout << "val this after:" << *(std::get<0>(*(this -> m_ptr))) << std::endl;
    			// std::cout << "val after:" << *(std::get<0>(*(tmp))) << std::endl;
    			return zipped_iterator(tmp) ; 
    		}  

    		// Prefix increment
    		zipped_iterator& operator++() 
    		{
    			tuple_increment(*m_ptr, 1);
    			return *this; 
    		}  

    		// Postfix increment
    		zipped_iterator operator++(int) 
    		{ 
    			zipped_iterator tmp = *this;
    			++(*this);
    			return tmp; 
    		}

    		bool operator== (const zipped_iterator& b) 
    		{
    			return tuple_equal(*m_ptr, b); 
    		};
    		bool operator!= (const zipped_iterator& b) 
    		{
    			// std::cout << "val this:" << *(std::get<0>(*(this -> m_ptr))) << std::endl;
    			// std::cout << "val b:" << *(std::get<0>(*(b.m_ptr))) << std::endl;

    			// std::cout << "tuple_equal: " << (tuple_equal(*m_ptr, *(b.m_ptr))?"true":"false") << std::endl;
    			return ! tuple_equal(*m_ptr, *(b.m_ptr)); 
    			// return tuple_not_equal(*m_ptr, b);
    		};

		private:

    		pointer m_ptr;
		};

	public:

		/**
		 * @brief      This class encapsulates two or more iterable objects to be able to iterate
		 *             both at the same time. Similar to python's zip. Direct use of this class is discourage
		 *             instead, use the function "zip_iters".
		 *
		 * @param      tb    tuple of 'begin's (aka first element of the iteration)
		 * @param      te    Tuple of 'end's (aka last element to iterate to)
		 */
    	zipped_t(std::tuple<Tn...>& tb, std::tuple<Tn...>& te)
    	{
    		// create_tuples(ts..., te...);
    		zipped_start = tb;
    		zipped_end   = te;
    	}

    	/**
    	 * @brief      The first element (which contains the first iterators of the encapsulated classes)
    	 *
    	 * @return     A zipped iterator.
    	 */
    	inline zipped_iterator begin()
		{	
		// std::cout << "begin: " << *(std::get<0>(zipped_start)) << " " << *(std::get<1>(zipped_start))<< std::endl;
			return zipped_iterator(&zipped_start);
		}

		/**
		 * @brief      The last element (which contains the last iterators of the encapsulated classes)
		 *
		 * @return     A zipped iterator.
		 */
		inline zipped_iterator end()
		{
			return zipped_iterator(&zipped_end);
		}

    template<std::size_t I = 0, typename... Tp>
	static inline typename std::enable_if< I == sizeof...(Tp), void>::type
  	tuple_begin_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<Tp*...>& t)
  	{ 
  		// return std::make_tuple(std::get<I>(t).begin());
  		// std::get<I>(tb) = std::move(std::get<I>(t).begin());
  	}

  	template<std::size_t I = 0, typename... Tp>
	static inline typename std::enable_if< I == sizeof...(Tp), void>::type
  	tuple_begin_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<std::shared_ptr<Tp>...>& t)
  	{ 
  	}

    template<std::size_t I = 0, typename... Tp>
    static inline typename std::enable_if< I < sizeof...(Tp), void>::type // >
    tuple_begin_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<Tp*...>& t)
    {
  		// return std::tuple_cat(std::make_tuple(std::get<I>(t).begin()), tuple_begin_create<I + 1, Tp...>(t));
  		std::get<I>(tb) = std::move(std::get<I>(t) -> begin());
  		tuple_begin_create< I + 1, Tp...>(tb, t);

    }

    	template<std::size_t I = 0, typename... Tp>
    	static inline typename std::enable_if< I < sizeof...(Tp), void>::type // >
    	tuple_begin_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<std::shared_ptr<Tp>...>& t)
    	{
  			std::get<I>(tb) = std::get<I>(t) -> begin();
  			tuple_begin_create< I + 1, Tp...>(tb, t);
    	}

	    template<std::size_t I = 0, typename... Tp>
		static inline typename std::enable_if< I == sizeof...(Tp), void>::type
	  	tuple_end_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<Tp*...>& t)
	  	{ 
	  	}

	  	template<std::size_t I = 0, typename... Tp>
		static inline typename std::enable_if< I == sizeof...(Tp), void>::type
	  	tuple_end_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<std::shared_ptr<Tp>...>& t)
	  	{ 
	  	}

	    template<std::size_t I = 0, typename... Tp>
	    static inline typename std::enable_if< I < sizeof...(Tp), void>::type // >
	    tuple_end_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<Tp*...>& t)
	    {
	  		// return std::tuple_cat(std::make_tuple(std::get<I>(t).begin()), tuple_begin_create<I + 1, Tp...>(t));
	  		std::get<I>(tb) = std::get<I>(t) -> end();
	  		tuple_end_create< I + 1, Tp...>(tb, t);

	    }

	    template<std::size_t I = 0, typename... Tp>
	    static inline typename std::enable_if< I < sizeof...(Tp), void>::type // >
	    tuple_end_create(std::tuple<typename Tp::iterator...>& tb, std::tuple<std::shared_ptr<Tp>...>& t)
	    {
	  		std::get<I>(tb) = std::get<I>(t) -> end();
	  		tuple_end_create< I + 1, Tp...>(tb, t);
	    }


		template<std::size_t I = 0, typename... It>
		static inline typename std::enable_if< I == sizeof...(It), void>::type
	  	tuple_unzip_vals(std::tuple<typename std::iterator_traits<It>::value_type...>& tz, std::tuple<It...>& t)
	  	{ 
	  	}

	    template<std::size_t I = 0, typename... It>
	    static inline typename std::enable_if< I < sizeof...(It), void>::type // >
	    tuple_unzip_vals(std::tuple<typename std::iterator_traits<It>::value_type...>& tz, std::tuple<It...>& t)
	    {
	  		// return std::tuple_cat(std::make_tuple(std::get<I>(t).begin()), tuple_begin_create<I + 1, Tp...>(t));
	  		std::get<I>(tz) = *std::get<I>(t);
	  		tuple_unzip_vals< I + 1, It...>(tz, t);
	    }
    private:
    	std::tuple<Tn...> zipped_start;
    	std::tuple<Tn...> zipped_end;
    
    };




    /**
     * Helper function to create the zipped iterator.
     * @params Tp& Any number of iterable objects
     * @returns Iterable object, can be unpacked using the unzip_vals functions alongside with std::tie.
     */
	template<typename... Tp> 
	zipped_t<typename Tp::iterator...> zip_iters(Tp&... t)
	{
		auto tp = std::make_tuple(&t...);
		std::tuple<typename Tp::iterator...> tb;
		std::tuple<typename Tp::iterator...> te;
		zipped_t<typename Tp::iterator...>::tuple_begin_create(tb, tp);
		zipped_t<typename Tp::iterator...>::tuple_end_create(te, tp);
		return zipped_t<typename Tp::iterator...>(tb, te);
	}

	template<typename... Tp> 
	zipped_t<typename Tp::iterator...> zip_iters(const std::shared_ptr<Tp>&... t)
	{
		auto tp = std::make_tuple(t...);
		std::tuple<typename Tp::iterator...> tb;
		std::tuple<typename Tp::iterator...> te;
		zipped_t<typename Tp::iterator...>::tuple_begin_create(tb, tp);
		zipped_t<typename Tp::iterator...>::tuple_end_create(te, tp);
		return zipped_t<typename Tp::iterator...>(tb, te);
	}

	/**
     * Helper function to unpack the iterator.
     * @params Tp& The zipped object containing 
     * @returns Iterable object, can be unpacked using the unzip_vals functions alongside with std::tie.
     */
	template<typename... It>
	std::tuple<typename std::iterator_traits<It>::value_type...> unzip(std::tuple<It...>& t)
	{
		std::tuple<typename std::iterator_traits<It>::value_type...> tb;
		zipped_t<typename std::iterator_traits<It>::value_type...>::tuple_unzip_vals(tb, t);
		return tb;
	}


	// template<class It1, class It2, class It3,
	// 	class T1 = typename std::iterator_traits<It1>::value_type, 
	// 	class T2 = typename std::iterator_traits<It2>::value_type,
	// 	class T3 = typename std::iterator_traits<It3>::value_type> 
	// std::tuple<T1, T2, T3> unzip(std::tuple<It1, It2, It3> t)
	// {
	// 	return std::make_tuple(*std::get<0>(t), *std::get<1>(t), *std::get<2>(t));
	// }

}