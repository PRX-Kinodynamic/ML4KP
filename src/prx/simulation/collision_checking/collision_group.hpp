#pragma once

/**
* @file collision_group.hpp
* @brief <b> [INCOMPLETE] A class responsible for keeping track of collision information. </b>
* @author Zakary Littlefield, Aravind Sivaramakrishnan, Troy McMahon
* */

#include "prx/simulation/plant.hpp"

#include <vector>

namespace prx
{
	/**
	 * The collision group keeps track of collision information, and is responsible 
	 * for collision checking.
	 * 
	 * For analytical plants, this is done by caching all pairs of geometries that could
	 * be in collision during planning. These include rigid bodies of robots as well as obstacles
	 * in the planning scene. Self-collisions are not accounted for. Online, a call to PQP
	 * is made to query for collisions.
	 * 
	 * For Bullet-simulated plants the collision group just keeps track of the different plants
	 * in the world. Internally, Bullet keeps track of collisions that haven't been excluded explicitly.
	 * This information is queried during a collision checking call.
	 * 
	 * @brief <b> A class responsible for keeping track of collision information. </b>
	 * @author Zakary Littlefield, Aravind Sivaramakrishnan, Troy McMahon
	 * */
	class collision_group_t
	{
	public:
		/**
		 * @brief Initializes the collision group.
		 * @param in_plants A vector of plants that are to be included for collision checking
		 * @param in_obstacles A vector of movable objects that are to be included for collision checking
		 * */
		collision_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles={});
		~collision_group_t();

		/**
		 * @brief A structure that stores distances and closest points for each collision pair.
		 * */
		struct pqp_distance_t
		{
			/**
			 * @brief A vector of distances between each collision pair in the collision cache.
			 * */
			std::vector<double> distances;
			/**
			 * @brief A vector of the closest point for each collision pair in the collision cache.
			 * 
			 * For every element of the collision pair, the closest point corresponds to the point on the
			 * first element of the collision pair that is closest to the second element of the collision pair.
			 * By convention, when constructing the collision cache, the second element of each pair typically 
			 * corresponds to a rigid body on a robot, while the first element corresponds to a rigid body that is
			 * considered to be an obstacle.
			 * 
			 * As a result, this is a vector of the closest point on each obstacle for each rigid body present on the robot.
			 * */
			std::vector<std::vector<double>> closest_points;
		};

		/**
		 * @brief Check if there is a collision in the scene.
		 * */
		virtual bool in_collision();
		/**
		 * @brief Returns the distance between all pairs of geometries in the scene.
		 * 
		 * For all pairs of geometries in the collision cache, a vector of distances and closest points is returned.
		 * Currently implemented only for analytical systems.
		 * 
		 * @return A structure of type pqp_distance_t
		 * 
		 * */
		pqp_distance_t get_distances();

	  
	protected:

		/** @brief A structure that stores geometry information for PQP queries.*/
		struct pqp_info_t
		{
			/** @brief The (x,y,z) position of the center of mass. */
			double pos[3];
			/** @brief The rotation matrix of the center of mass. */
			double rot[3][3];
			std::weak_ptr<transform_t> transform;
			std::weak_ptr<PQP_Model> model;

			/** @brief Updates the poses for the geometry. */
			void update_info();
		};

		/** @brief Updates the configuration for all the plants in the scene. */
		void update_plants();

		/** @brief The collision cache. */
		std::vector<std::pair<std::weak_ptr<pqp_info_t>, std::weak_ptr<pqp_info_t>>> collision_cache;

		/** @brief A list of all plants in the world. */
		std::vector<system_ptr_t> plants;
		
		/** @brief A list of all geometries in the world. */
		std::vector<std::shared_ptr<pqp_info_t>> infos;
	// private: 
		collision_group_t(){};
	};
}
