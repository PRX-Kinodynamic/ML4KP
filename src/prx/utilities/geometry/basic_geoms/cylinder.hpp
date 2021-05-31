#pragma once
/**
* @file cylinder.hpp
* @brief <b> A cylinder object.</b>
* 
* @author Zakary Littlefield
*/

#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
	/**
	* @brief <b> A cylinder object.</b>
    * 
    * @author Zakary Littlefield
    */
	class cylinder_t : public movable_object_t
	{
		public:
		/**
		* @brief <b> Constructor.</b>
		* 
		* Initializes a cylinder-shaped rigid body.
		*
		* @param object_name Name of the box.
		* @param radius Radius of the base of the cylinder.
		* @param height Height of the cylinder.
		* @param pose The pose of the cylinder.
		*
		*/
		
		cylinder_t(const std::string& object_name, double radius, double height, const transform_t& pose);
		virtual ~cylinder_t(){}
	};
}