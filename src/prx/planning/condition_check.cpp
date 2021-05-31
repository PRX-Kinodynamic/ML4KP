

#include "prx/utilities/defs.hpp"
#include "prx/planning/condition_check.hpp"

namespace prx
{
	condition_check_t::condition_check_t(std::string type, double check )
	{
		condition_check = check;
		if(type=="iterations")
			condition_type = 0;
		else if(type=="time")
			condition_type = 1;
		else
		{
			prx_throw("Condition type is invalid!");
		}
		iteration_counter=0;
	}
	void condition_check_t::reset()
	{
		timer.reset();
		iteration_counter = 0;
	}

	bool condition_check_t::check()
	{
		++iteration_counter;
		if(condition_type==0)
		{
			if(iteration_counter>=condition_check)
				return true;
		}
		else if(condition_type==1)
		{
			if(timer.measure()>=condition_check)
			{
				return true;
			}
		}
		return false;
	}	
}