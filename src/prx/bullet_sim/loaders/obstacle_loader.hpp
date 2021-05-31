#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/bullet_sim/bullet_defs.hpp"
#include "prx/utilities/geometry/movable_object.hpp"

#include "RobotSimulator/b3RobotSimulatorClientAPI.h"

namespace prx
{
	// TODO: Should we create a class instead?
	// TODO: Add a way to load from different path
  std::pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>> load_obstacles(std::string obstacles_file,b3RobotSimulatorClientAPI* sim=NULL);
}
