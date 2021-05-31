#pragma once

/**
* @file sphere.hpp
* @brief <b> A spherical object.</b>
* 
* @author Zakary Littlefield
*/

#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
	/**
	* @brief <b> A spherical object.</b>
	* 
	* @author Zakary Littlefield
	*/
	class sphere_t : public movable_object_t
	{
	public:
		/**
		* @brief <b> Constructor.</b>
		* 
		* Initializes a spherical rigid body.
		*
		* @param object_name Name of the sphere.
		* @param radius Radius of the sphere.
		* @param pose The pose of the box.
		*
		*/
		sphere_t(const std::string& object_name, double radius, const transform_t& pose);
		virtual ~sphere_t(){}
	};
}