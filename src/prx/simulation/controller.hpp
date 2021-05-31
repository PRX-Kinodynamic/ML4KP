#pragma once

#include "prx/simulation/system.hpp"
#include "prx/simulation/playback/plan.hpp"
#include "prx/simulation/playback/trajectory.hpp"

#include <unordered_map>

namespace prx
{
	// TODO: Change name?
	// Desired states/points
	// Objective
	// Goal
	// local_goal
	class set_points_t
	{
	public:
		set_points_t(const space_t* _space)
		{
			space = _space;
		}

		inline space_point_t operator[](unsigned index) const
		{
			prx_assert(index < set_points.size(), "Set point out of bounds. Size: " << set_points.size() << " requested: " << index);
			return set_points[index];
		}

		inline space_point_t& operator[](unsigned index)
		{
			prx_warn_cond(index <= set_points.size(), "Adding " << (index - set_points.size()) << " set_points");
			for (int i = set_points.size(); i <= index; ++i)
			{
				set_points.push_back( space -> make_point());
			}
			return set_points[index];

		}
	private:
		std::vector<space_point_t> set_points;
		const space_t* space;
	};

	class controller_t
	{
	public:
		controller_t(system_ptr_t _plant, std::string _name = "base_controller") : set_points(_plant -> get_state_space())
		{
			plant = _plant;
			name = _name;
			// set_points = set_points_t(plant -> get_state_space());
		}
		virtual ~controller_t();

		virtual void compute_controls()=0;

		// wrapper functions for better readability
		
		inline const space_t* get_state_space() const
		{
			return plant -> get_state_space();
		}
		inline const space_t* get_control_space() const
		{
			return plant -> get_control_space();
		}

		virtual void propagate(const double simulation_step)
		{
			plant -> propagate(simulation_step);
		}
		virtual void propagate(const double simulation_step, const propagate_step step)
		{
			plant -> propagate(simulation_step, step);
		}

		virtual void set_plan(const plan_t& _plan)
		{
			plan = std::make_shared<plan_t>(_plan);
		}

		virtual std::shared_ptr<plan_t> get_plan()
		{
			return plan;
		}

		virtual void init_plan()
		{
			plan = std::make_shared<plan_t>(get_control_space());
		}

		set_points_t set_points;



	protected:
		
		system_ptr_t plant;
		std::string name;

		std::shared_ptr<plan_t> plan; // Control sequence

	};
}
