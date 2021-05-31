#pragma once
/**
 * @brief <b> An abstract node class. </b>
 * @file abstract_node.hpp
 * @author Zakary Littlefield
*/

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/data_structures/gnn.hpp"

namespace prx
{
	/** @brief Node index. */
	typedef long unsigned int node_index_t;

	/**
	 * @brief <b> An abstract node class. </b>
	 * 
	 * @author Zakary Littlefield
	*/
	class abstract_node_t : public proximity_node_t
	{
	public:
		abstract_node_t() : proximity_node_t(){}
		virtual ~abstract_node_t(){}

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

		/**
		 * @brief The space point associated with this node.
		 * */
		space_point_t point;
	};
}