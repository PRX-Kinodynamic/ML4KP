#pragma once
/**
* @file box.hpp
* @brief <b> A box object.</b>
* 
* @author Zakary Littlefield
*/

#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
	/**
    * @brief <b> A box object.</b>
    * 
    * @author Zakary Littlefield
    */
	class box_t : public movable_object_t
	{
	public:
		/**
		* @brief <b> Constructor.</b>
		* 
		* Initializes a box-shaped rigid body.
		*
		* @param object_name Name of the box.
		* @param dim_x Length along the x-axis.
		* @param dim_y Length along the y-axis.
		* @param dim_z Length along the z-axis.
		* @param pose The pose of the box.
		*
		*/
		box_t(const std::string& object_name,double dim_x, double dim_y, double dim_z, const transform_t& pose);
		virtual ~box_t(){}
	};
}