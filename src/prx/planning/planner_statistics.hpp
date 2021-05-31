#pragma once

#include "prx/planning/planners/planner.hpp"

namespace prx
{
	// TODO: Maybe change name to something like stats_gatherer...?
	class planner_statistics_t
	{
	public:
		planner_statistics_t();
		~planner_statistics_t();

		void link_planner(planner_t* _planner);
		void link_criterion(condition_check_t* _check);

		void repeat_data_gathering(int repeats, bool verbose = true);

		std::string serialize_header();

		std::string serialize();

	protected:
		planner_t* planner;
		condition_check_t* check;
		std::vector<std::vector<double>> all_stats;

	};
}
