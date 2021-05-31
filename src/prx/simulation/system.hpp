#pragma once

#include "prx/utilities/defs.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/simulation/system_factory.hpp"

#include <memory>

namespace prx
{

	extern double simulation_step;
	class controller_t;

	class system_t;
	// class system_ptr_t;
	typedef std::shared_ptr<system_t> system_ptr_t;

	template<class T>
	system_ptr_t create_system(const std::string& path)
	{
		system_ptr_t new_ptr;
		new_ptr.reset(new T(path));
		return new_ptr;
	}

	bool is_child_system(system_ptr_t parent, system_ptr_t child);


	class system_t : public std::enable_shared_from_this<system_t>
	{
	public:
		system_t(const std::string& path);
		virtual ~system_t();

		virtual inline const space_t* get_state_space() const
		{
			return state_space;
		}
		inline const space_t* get_control_space() const
		{
			return input_control_space;
		}

		virtual void add_system(system_ptr_t& )=0;

		// Keeping this for analitical plants. 
		virtual void propagate(const double simulation_step) = 0;
		virtual void propagate(const double simulation_step, const propagate_step step)
		{
			// The defaul implementation is not using propagate_step
			// Currently propagate step is only used inside bullet
			// It can always be overridden if necessary
			propagate(simulation_step);
		}

		virtual void compute_control()=0;

		virtual void compute_stopping_maneuver(space_point_t, std::vector<double>&, std::vector<double>&);
		virtual void finalize_system_tree()
		{
			//default do nothing because you don't have any subsystems
		}

        virtual void set_state_space_bounds(const std::vector<double>& lower,const std::vector<double>& upper)=0;


		inline std::string get_pathname()
		{
			return pathname;
		}

		inline plant_type get_system_type()
		{
			return system_type;
		}

		space_t* state_space;
		space_t* input_control_space;
	protected:

		std::weak_ptr<system_t> parent_system;

		void set_parent_system(system_ptr_t parent)
		{
			if(!parent_system.expired())
				prx_throw("Trying to add system "<<pathname<<" underneath "<<parent->get_pathname()<<" when "<<parent_system.lock()->get_pathname()<<" is already its parent.");
			parent_system = parent;
		}

		std::string pathname;
		plant_type system_type;

		std::vector<double*> state_memory;
		std::vector<double*> control_memory;

	private:
		system_t(){}
		friend controller_t;

	};
}
