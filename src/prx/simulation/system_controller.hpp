#pragma once

#include "prx/simulation/system.hpp"

#include <unordered_map>

namespace prx
{

	class system_controller_t : public system_t
	{
	public:
		system_controller_t(const std::string& path);
		virtual ~system_controller_t();

		virtual inline const space_t* get_state_space() const override final
		{
			return composite_state_space;
		}

		virtual void add_system(system_ptr_t& ) override final;

		virtual void compute_control() override;

		virtual void propagate(const double simulation_step) override;

		virtual void finalize_system_tree() override;

	protected:
		space_t* composite_state_space;

		std::unordered_map<std::string, system_ptr_t> subsystems;

	};
}