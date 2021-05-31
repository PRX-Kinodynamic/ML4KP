#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/simulation/system.hpp"


namespace prx
{
	const std::string bullet_path = std::string(BULLET_PHYSICS_PATH);
	template<class T>
	system_ptr_t create_system(const std::string& path,std::vector<double> start_state)
	{
		system_ptr_t new_ptr;
		new_ptr.reset(new T(path,start_state));
		return new_ptr;
	}
}