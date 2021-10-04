#pragma once
/**
 * @file condition_check.hpp
 * @brief A class which checks if a condition is met.
 * @details A class which checks if a condition is met.
 * @author Zakary Littlefield, Aravind Sivaramakrishnan
 */

#include <string>

#include "prx/utilities/general/timer.hpp"

namespace prx
{
	// TODO: Add condition of "Return at first solution found"
	/**
	 * @brief A class which checks if a condition is met.
	 * @details A class which checks if a condition is met. 
	 * 
	 * Two types of conditions can be specified: iterations and time.
	 * 
	 * @author Zakary Littlefield, Aravind Sivaramakrishnan
	 */
	class condition_check_t
	{
	public:
		condition_check_t(std::string type, double check );

		/**
		 * @brief Resets the internal counts and timers.
		 * @details Resets the internal counts and timers.
		 */
		void reset();

		/**
		 * @brief Check if condition is satisfied.
		 * @details Check if condition is satisfied.
		 * @return True if satisfied, false if not.
		 */
		bool check();

		/**
		 * @brief Report that a new solution has been found.
		 * @details The planner calls this function when it has found a new solution.
		 */
		void report_new_solution();

		/**
		 * @brief Get the time on the timer.
		 * @details Get the time on the timer.
		 * 
		 * @return The current time.
		 */
		double time()
		{
			return timer.measure();
		}

		/**
		 * @brief Get the current iterations.
		 * @details Get the current iterations.
		 * 
		 * @return The current iterations.
		 */
		long unsigned iterations()
		{
			return iteration_counter;
		}

		/**
		 * @brief Get the max time or iterations.
		 * @details Get the max time or iterations.
		 * 
		 * @return The max time or iterations.
		 */
		long unsigned get_check_value()
		{
			return condition_check;
		}

	protected:

		/**
		 * @brief The timer used for checking times.
		 */
		timer_t timer;

		/**
		 * @brief A counter for iterations.
		 */
		long unsigned iteration_counter;

		/**
		 * @brief A counter for the number of solutions found.
		 */
		long unsigned solution_counter;

		/**
		 * @brief The maximum time or iterations before being satisfied.
		 */
		double condition_check;

		/**
		 * @brief Which type of condition to check. 0 for iterations, 1 for time.
		 */
		unsigned condition_type;

	};
}
