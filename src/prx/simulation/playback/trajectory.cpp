
#include "prx/simulation/playback/trajectory.hpp"

namespace prx
{
	trajectory_t::trajectory_t(const space_t* space) :state_space(space)
	{
		//allocate a number of initial points
		for( unsigned i = 0; i < 4; i++ )
		{
			auto point = state_space->make_point();
			states.push_back(point);
		}
		num_states = 0;
		max_num_states = 4;
		end_iterator = states.begin();
		const_end_iterator = states.begin();
	}
	trajectory_t::~trajectory_t()
	{
		states.clear();
	}
	trajectory_t::trajectory_t(const trajectory_t& traj)
	{
		state_space = traj.state_space;
		num_states = 0;
		max_num_states = 0;
		end_iterator = states.begin();
		const_end_iterator = states.begin();
		(*this) = traj;
	}
	void trajectory_t::resize(unsigned num_size)
	{
		while( max_num_states < num_size )
		{
			increase_buffer();
		}
		num_states = num_size;
		end_iterator = states.begin();
		const_end_iterator = states.begin();
		std::advance(end_iterator, num_states);
		std::advance(const_end_iterator, num_states);
	}
	trajectory_t& trajectory_t::operator=(const trajectory_t& t)
	{
		prx_assert(state_space->get_space_name()==t.state_space->get_space_name(),"Trajectory buffers have mismatched state spaces.");

		if( this != &t )
		{
			//check the size of the two trajectories
			while( max_num_states < t.num_states )
			{
				increase_buffer();
			}
			//clear the trajectory
			clear();

			for(const auto& state : t)
			{
				state_space->copy_point(*end_iterator, state);
				++num_states;
				++end_iterator;
				++const_end_iterator;
			}
		}
		return *this;

	}

	bool trajectory_t::operator==(const trajectory_t& t)
	{
		prx_assert(state_space->get_space_name()==t.state_space->get_space_name(),"Trajectory buffers have mismatched state spaces.");

		if (t.size() != this -> num_states)
			return false;

		for(double i = 0; i < this -> num_states; i++)
		{
			if(t.at(i) != this -> at(i))
				return false;
		}

		return true;

	}
	bool trajectory_t::operator!=(const trajectory_t& t)
	{
		return !(*this == t);
	}
	
	trajectory_t& trajectory_t::operator+=(const trajectory_t& t)
	{
		prx_assert(state_space->get_space_name()==t.state_space->get_space_name(),"Appending trajectory from different space.");
		unsigned new_size = t.num_states + num_states;
		while( max_num_states < new_size )
		{
			increase_buffer();

			end_iterator = states.begin();
			const_end_iterator = states.begin();
			std::advance(end_iterator, num_states);
			std::advance(const_end_iterator, num_states);
		}

		for(auto&& state : t)
		{
			state_space->copy_point(*end_iterator, state);
			++num_states;
			++end_iterator;
			++const_end_iterator;
		}
		return *this;
	}
	void trajectory_t::clear()
	{
		end_iterator = states.begin();
		const_end_iterator = states.begin();
		num_states = 0;
	}
	
	void trajectory_t::copy_onto_back(space_point_t state)
	{
		if( (num_states + 1) >= max_num_states )
		{
			increase_buffer();

			end_iterator = states.begin();
			const_end_iterator = states.begin();
			std::advance(end_iterator, num_states);
			std::advance(const_end_iterator, num_states);

		}
		state_space->copy_point(*end_iterator, state);
		++end_iterator;
		++const_end_iterator;
		++num_states;
	}

	void trajectory_t::copy_onto_back(const space_t* space)
	{
		prx_assert(state_space->get_space_name()==space->get_space_name(),"Trying to add a new point from the wrong space ("+space->get_space_name()+" in a trajectory with space"+state_space->get_space_name()+")");

		if( (num_states + 1) >= max_num_states )
		{
			increase_buffer();

			end_iterator = states.begin();
			const_end_iterator = states.begin();
			std::advance(end_iterator, num_states);
			std::advance(const_end_iterator, num_states);

		}
		space->copy_to_point(*end_iterator);
		++end_iterator;
		++const_end_iterator;
		++num_states;
	}

    std::string trajectory_t::print(unsigned precision) const
    {
        std::stringstream out(std::stringstream::out);

        int counter = 0;

        for(const space_point_t& st : *this)
        {
            out << counter << ": [" << state_space->print_point(st, precision)
                    << "]" << std::endl;
            counter++;
        }
        return out.str();
    }


	space_point_t trajectory_t::interpolate(double s) const
	{
		s = (s<0?0:s);
		s = (s>1?1:s);

		const double closest_index = s*(num_states-1);//this scales into the index space of the traj
		const unsigned lower_index = std::floor(closest_index);
		if(lower_index==(num_states-1))//handle the border case
			return back();
		const double interpolation_value = closest_index-lower_index;
		auto inter_point = state_space->make_point();
		state_space->interpolate(states[lower_index],states[lower_index+1],interpolation_value,inter_point);
		return inter_point;
	}


	void trajectory_t::increase_buffer()
	{
		if( max_num_states != 0 )
		{
			states.resize(2*max_num_states);
			for( unsigned i = max_num_states; i < 2*max_num_states; i++ )
			{
				states[i]=state_space->make_point();
			}
			max_num_states *= 2;
		}
		else
		{
			auto point = state_space->make_point();
			states.push_back(point);
			max_num_states = 1;
			num_states = 0;
			end_iterator = states.begin();
			const_end_iterator = states.begin();
		}

	}
}