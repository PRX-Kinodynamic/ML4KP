#pragma once

/**
 * @brief <b> An abstract edge class. </b>
 * @file abstract_edge.hpp
 * @author Zakary Littlefield
*/

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"

namespace prx
{
	/** @brief Edge index. */
	typedef long unsigned int edge_index_t;

	/**
	 * @brief <b> An abstract edge class. </b>
	 * 
	 * @author Zakary Littlefield
	*/
	class abstract_edge_t
	{
	public:
		abstract_edge_t(){}
		virtual ~abstract_edge_t(){}

		template<class T>
		const T* as() const
		{
			return static_cast<const T*>(this);
		}

		template<class T>
		T* as()
		{
			return static_cast<T*>(this);
		}
	};
}