#include "prx/simulation/system_group_manager.hpp"

namespace prx
{
	system_group_manager_t::system_group_manager_t()
	{

	}
	system_group_manager_t::~system_group_manager_t()
	{

	}

	void system_group_manager_t::add_system_group(const std::string& group_name, const std::vector<system_ptr_t>& systems)
	{
		system_groups[group_name] = std::make_shared<system_group_t>(systems);
	}

	std::shared_ptr<system_group_t> system_group_manager_t::get_system_group(const std::string& group_name)
	{
		prx_assert(system_groups.count(group_name)!=0,"Trying to get a system group with name "<<group_name<<" that hasn't been built");
		return system_groups[group_name];
	}
}