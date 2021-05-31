#pragma once

#include "prx/utilities/defs.hpp"

#include "prx/simulation/system.hpp"

#include "prx/simulation/system_group.hpp"

#include <unordered_map>

namespace prx
{
	class system_group_manager_t
	{
	public:
		system_group_manager_t();
		~system_group_manager_t();

		void add_system_group(const std::string& group_name, const std::vector<system_ptr_t>& systems);
		std::shared_ptr<system_group_t> get_system_group(const std::string& group_name);
	private:
		std::unordered_map<std::string,std::shared_ptr<system_group_t>> system_groups;
	};
}
