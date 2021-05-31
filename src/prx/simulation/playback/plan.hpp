#pragma once

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/defs.hpp"

#include <deque>

namespace prx
{
	struct plan_step_t
	{
		plan_step_t(space_point_t control, const double duration)
			: control(control), duration(duration){ }

		void copy_step(const space_t* space, const plan_step_t& step)
		{
			space->copy_point(control,step.control);
			duration = step.duration;
		}

		space_point_t control;
		double duration;
	};
	/**
     * @brief <b>A class that defines a plan.</b>
	 * 
	 * A plan consists of a sequence of piecewise constant controls, each of which are
	 * applied for a specified duration. Each (control, duration) pair is referred to as a <i> step </i>.
	 * 
     * @authors Zakary Littlefield
     * 
     */
	class plan_t
	{
	public:
		typedef std::deque<plan_step_t>::iterator iterator;
		typedef std::deque<plan_step_t>::const_iterator const_iterator;

		plan_t(const space_t* new_space);

		plan_t(const plan_t& other);

		~plan_t();

		/**
         * @brief Returns the number of steps in the plan.
		 * 
		 * A plan {(u_1,t_1),(u_2,t_2),...,(u_M,t_M)} has <i> M </i> steps.
         * @return Number of steps in the plan.
         */
		inline unsigned size() const
		{
			return num_steps;
		}
		/**
         * @brief Returns the duration of the plan.
		 * 
		 * For a plan {(u_1,t_1),(u_2,t_2),...,(u_M,t_M)}, it's duration is given by
		 * t_1 + t_2 + ... + t_M.
		 * 
         * @return Duration of the plan.
         */
		inline double duration() const
		{
			double t = 0;
			for(auto&& step : *this)
			{
				t+=step.duration;
			}
			return t;
		}

		inline const plan_step_t& operator[](unsigned index) const
		{
			prx_assert(index < num_steps,"Trying to access plan outside of bounds.");
			return steps[index];
		}

		inline const plan_step_t& at(unsigned index) const
		{
			prx_assert(index < num_steps,"Trying to access plan outside of bounds.");
			return steps[index];
		}

		inline space_point_t operator[](double t) const
		{
			return at(t);
		}

		inline space_point_t at(double t) const
		{
			for(auto&& step : steps)
			{
				if(t<step.duration)
					return step.control;
				else
					t-=step.duration;
			}
			prx_throw("Indexed into plan with time outside the plan's full duration.");
		}

		inline plan_step_t& front()
		{
			prx_assert(num_steps != 0,"Trying to access the front of an empty plan.");
			return steps[0];
		}

		inline plan_step_t& back()
		{
			prx_assert(num_steps != 0,"Trying to access the back of an empty plan.");
			return steps[num_steps - 1];
		}

		inline iterator begin()
		{
			return steps.begin();
		}

		inline iterator end()
		{
			return end_iterator;
		}

		inline const_iterator begin() const
		{
			return steps.begin();
		}

		inline const_iterator end() const
		{
			return const_end_iterator;
		}

		void resize(unsigned num_size);

		plan_t& operator=(const plan_t& t);

		plan_t& operator+=(const plan_t& t);

		/**
         * @brief Clear the contents of the plan.
         */
		void clear();

		void copy_to(const double start_time, const double duration, plan_t& t);

		void copy_onto_back(space_point_t control, double time);

		void copy_onto_front(space_point_t control, double time);

		/**
         * @brief Add one step to the front of the plan.
		 * @param time Duration of the first control of the modified plan.
         */
		void append_onto_front(double time);

		/**
         * @brief Add one step to the back of the plan.
		 * @param time Duration of the last control of the modified plan.
         */
		void append_onto_back(double time);

		/**
         * @brief Extend the duration of the last control in the plan by a specified time.
		 * @param time Time to extend the duration of the last control by (in seconds).
         */
		void extend_last_control(double time);

		/**
         * @brief Removes the first control from the plan.
         */
		void pop_front();

		/**
         * @brief Removes the last control from the plan.
         */
		void pop_back();

        /**
         * @brief Helper function to print out the plan.
		 * @return A string object that outputs the plan.
         */
		std::string print( unsigned precision = 3 ) const;

	private:

		void increase_buffer();

		const space_t* control_space;

		iterator end_iterator;

		const_iterator const_end_iterator;

		unsigned num_steps;

		unsigned max_num_steps;

		std::deque<plan_step_t> steps;
	};
}
