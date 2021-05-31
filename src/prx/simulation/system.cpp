#include "prx/simulation/system.hpp"

namespace prx
{
	double simulation_step;

	bool is_child_system(system_ptr_t parent, system_ptr_t child)
	{
		std::string p_path = parent->get_pathname();
		std::string c_path = child->get_pathname();
		while(p_path.size()<c_path.size())
		{
			auto s_paths = reverse_split_path(c_path);
			if(reverse_string_compare(p_path,s_paths.first))
				return true;
			if(c_path == s_paths.first)
				return false;
			c_path = s_paths.first;
		}
		return false;
	}

	system_t::system_t(const std::string& path)
	{
		state_space = nullptr;
		input_control_space = nullptr;
		pathname = path;
		system_type = plant_type::ANALYTICAL;
	}

	system_t::~system_t()
	{
		if(state_space!=nullptr)
		{
			delete state_space;
		}
		if(input_control_space!=nullptr)
		{
			delete input_control_space;
		}
		parent_system.reset();
	}

	void system_t::compute_stopping_maneuver(space_point_t start_state, std::vector<double>& times, std::vector<double>& ctrls)
	{
		//the control should have already been set in the vehicle space
	}
}
