#ifndef BULLET_NOT_BUILT

#pragma once
#include <unistd.h>

#include "prx/bullet_sim/bullet_defs.hpp"

#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/simulation/loaders/obstacle_loader.hpp"

#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "RobotSimulator/b3RobotSimulatorClientAPI.h"
#include "Bullet3Common/b3HashMap.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btDefaultMotionState.h"
#include "SharedMemory/RemoteGUIHelper.h"

namespace prx
{
	class bullet_t : public plant_t
	{
	public:
		bullet_t(const std::string& path);
		virtual ~bullet_t();
		
		void setup();

		b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();

	    btVector3 getEulerFromQuaternion(btQuaternion q);

	    btQuaternion getQuaternionFromEuler(const btVector3& rollPitchYaw);
	  
		virtual void propagate(const double simulation_step) override final;

		virtual void propagate(const double simulation_step, const propagate_step step) override final;

		virtual void compute_control() override;

		virtual void update_from_bullet(const space_point_t& point, const bool save_sim_state);

		virtual void update_to_bullet(const space_point_t& point);

		virtual void update_configuration() override final;

	  virtual void print_trajectories(const std::vector<trajectory_t> trajs);

		virtual void visualize_trajectories(const std::vector<trajectory_t> trajs);

		virtual void visualize_goal(const space_point_t goal, const double radius);

		virtual void purge_saved_states();

	  void execute_traj(trajectory_t traj);


		void add_exclusion(int bID1, int lID1, int bID2, int lID2);

		bool isCollision(bool b_include_bounding_box=true);

		bool b_exclude(std::pair<std::pair<int, int>, std::pair<int, int> > excluded_pair, const b3ContactPointData &contact);

		bool b_exclude_contact(const b3ContactPointData &contact);

	  void setBasePositionAndRotation(btVector3 basePosition, btVector3 baseRotation);

	  std::vector<double> state_bounds_l, state_bounds_u, control_bounds_l, control_bounds_u;

	  int plane_id, uniqueId,count;

		b3RobotSimulatorAddUserDebugLineArgs* lineArgs = new b3RobotSimulatorAddUserDebugLineArgs;

	protected:
		std::vector<double> state_vec, control_vec;

		b3RobotSimulatorSetPhysicsEngineParameters physicsArgs;

		int physicsClientId, lastSavedId;

		std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > m_CD_exclusion_list;

		virtual void compute_derivative() override final;



	  std::string control_topo;
	  std::string state_topo;
	};
}
#endif
