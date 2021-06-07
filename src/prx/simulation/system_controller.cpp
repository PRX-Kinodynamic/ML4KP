#include "prx/simulation/system_controller.hpp"

namespace prx
{
	system_controller_t::system_controller_t(const std::string& path) : system_t(path)
	{
		subsystems.clear();
		composite_state_space=nullptr;
	}
	system_controller_t::~system_controller_t()
	{
		if(composite_state_space!=nullptr)
		{
			delete composite_state_space;
		}
		subsystems.clear();
		
	}
	void system_controller_t::add_system(system_ptr_t& ptr)
	{
		auto split_result = reverse_split_path(ptr->get_pathname());
		if(reverse_string_compare(pathname,split_result.first))
		{
			//we have found the place to add this system
			subsystems[ptr->get_pathname()] = ptr;
			ptr->set_parent_system(shared_from_this());
		}
		else
		{
			bool added=false;
			for(auto&& subsystem : subsystems)
			{
				if(!added && is_prefix(subsystem.second->get_pathname(),ptr->get_pathname()))
				{
					added = true;
					add_system(ptr);
				}
			}
			if(!added)
			{
				prx_throw("Trying to add system with path "<<ptr->get_pathname()<<" into "<<pathname);
			}
		}
	}
	void system_controller_t::compute_control()
	{
		for(auto sys : subsystems)
		{
			sys.second->compute_control();
		}
	}
	void system_controller_t::propagate(const double simulation_step)
	{
		for(auto sys : subsystems)
		{
			sys.second->propagate(simulation_step);
		}
	}
	void system_controller_t::finalize_system_tree()
	{
		prx_assert(composite_state_space==nullptr,"Finalize_system_tree may have already been called on "<<pathname<<". The state space in this controller has already been composed.");
		//populate the state space of this controller by looking at the subsystems
		std::vector<const space_t*> all_spaces;
		all_spaces.push_back(state_space);
		for(auto&& sys : subsystems)
		{
			sys.second->finalize_system_tree();
			all_spaces.push_back(sys.second->get_state_space());
		}
		composite_state_space = new space_t(all_spaces);
	}
}