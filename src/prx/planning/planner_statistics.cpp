#include "prx/planning/planner_statistics.hpp"

namespace prx
{
	planner_statistics_t::planner_statistics_t()
	{

	}
	planner_statistics_t::~planner_statistics_t()
	{

	}

	void planner_statistics_t::link_planner(planner_t* _planner)
	{
		planner = _planner;
	}

	void planner_statistics_t::link_criterion(condition_check_t* _check)
	{
		check = _check;
	}

	void planner_statistics_t::repeat_data_gathering(int repeats, bool verbose)
	{
		for(int i=0;i<repeats;i++)
		{
			check->reset();
			planner->resolve_query(check);
			all_stats.push_back(planner->get_statistics());
			if (verbose) output_progress_bar(i*1.0/repeats);
		}
	}

	std::string planner_statistics_t::serialize_header()
	{
		std::stringstream ss;
		auto header = planner -> get_statistics_header();
		auto last = header.end();
		for (auto first = header.begin(); first != last; )
		{
			ss << *first;
			if (++first != last)
				ss << ",";
		}
		ss << std::endl;
		return ss.str();
	}

	std::string planner_statistics_t::serialize()
	{
		std::stringstream ss;
		for(auto& data_point : all_stats)
		{
			auto last = data_point.end();
			for (auto first = data_point.begin(); first != last; )
			{
				ss << *first;
				if (++first != last)
					ss << ",";
			}
			ss << "\n";
		}
		return ss.str();
	}
}