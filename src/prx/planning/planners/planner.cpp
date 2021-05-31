#include "prx/planning/planners/planner.hpp"

namespace prx
{
	// CONSTRUCTION, LINK_SPECIFICATION, PREPROCESS, LINK_QUERY, RESOLVE_QUERY, FULFILL_QUERY
	planner_t::planner_t(const std::string& new_name)
	{
		planner_name = new_name;
	}
	planner_t::~planner_t()
	{
	}
	void planner_t::link_and_setup_spec(planner_specification_t* spec)
	{
		if(planner_state != planner_stage_t::CONSTRUCTION)
			_reset();
		_link_and_setup_spec(spec);
		planner_state=planner_stage_t::LINK_SPECIFICATION;
	}
	// If this is necessary, let not create another function but simply overload it
	// Also, just use shared_ptr.get().
	// void planner_t::link_and_setup_spec_shared(std::shared_ptr<planner_specification_t> spec)
	// {
	// 	link_and_setup_spec(spec.get());
	// }
	bool planner_t::preprocess()
	{
		if(planner_state!=planner_stage_t::LINK_SPECIFICATION && planner_state!=planner_stage_t::PREPROCESS)
		{
			prx_throw("Preprocess can only be called after link_and_setup_spec or after preprocess. Current planner stage is "<<get_current_stage_name());
		}
		const auto res = _preprocess();
		planner_state = planner_stage_t::PREPROCESS;
		return res;

	}
	bool planner_t::link_and_setup_query(planner_query_t* query)
	{
		if(planner_state!=planner_stage_t::PREPROCESS && planner_state!=planner_stage_t::LINK_QUERY && planner_state!=planner_stage_t::RESOLVE_QUERY && planner_state!=planner_stage_t::FULFILL_QUERY)
		{
			prx_throw("Link and setup query can only be after preprocess has been called (any query function). Current planner stage is "<<get_current_stage_name());
		}
		const auto res = _link_and_setup_query(query);
		planner_state = planner_stage_t::LINK_QUERY;
		return res;
	}
	// bool planner_t::link_and_setup_query_shared(std::shared_ptr<planner_query_t> query)
	// {
	// 	return link_and_setup_query(query.get());
	// }
	void planner_t::resolve_query(condition_check_t* condition)
	{
		if(planner_state!=planner_stage_t::LINK_QUERY && planner_state!=planner_stage_t::RESOLVE_QUERY && planner_state!=planner_stage_t::FULFILL_QUERY)
		{
			prx_throw("Resolve query can only be called after link_and_setup_query, resolve_query, or fulfill_query. Current planner stage is "<<get_current_stage_name());
		}
		_resolve_query(condition);
		planner_state = planner_stage_t::RESOLVE_QUERY;
	}
	void planner_t::fulfill_query()
	{
		if(planner_state!=planner_stage_t::RESOLVE_QUERY)
		{
			prx_throw("Fulfill query can only be called after resolve_query. Current planner stage is "<<get_current_stage_name());
		}
		_fulfill_query();
		planner_state = planner_stage_t::FULFILL_QUERY;
	}

	void planner_t::reset()
	{
		planner_state = planner_stage_t::CONSTRUCTION;
		_reset();
	}

	std::string planner_t::get_current_stage_name()
	{
		switch(planner_state)
		{
			case planner_stage_t::CONSTRUCTION:
				return "CONSTRUCTION";
			case planner_stage_t::LINK_SPECIFICATION:
				return "LINK_SPECIFICATION";
			case planner_stage_t::PREPROCESS:
				return "PREPROCESS";
			case planner_stage_t::LINK_QUERY:
				return "LINK_QUERY";
			case planner_stage_t::RESOLVE_QUERY:
				return "RESOLVE_QUERY";
			case planner_stage_t::FULFILL_QUERY:
				return "FULFILL_QUERY";
			default:
				prx_throw("planner_stage_t stage error!")
				return "ERROR!";
		}
	}
}
