#pragma once

#include <memory>
#include <unordered_map>
#include <string>

#include "prx/simulation/system.hpp"

namespace prx
{

	class system_t;


	typedef std::shared_ptr<system_t> system_ptr_t;

	typedef std::function<system_ptr_t(std::string)> system_gen_fn ;
	typedef std::function<double(system_ptr_t)> velocity_gen_fn ;
	// typedef system_ptr_t (*system_gen_fn) (std::string, ...);
	class system_factory_t 
	{
	public:
		static system_factory_t& get();

		// TODO: remove default for path ==> done it for backwards compatibility
		// TODO: Change system to system

		/**
		 * When called, this methods creates an instance of the system associated to the given name and assigns the given path.
		 * @param  name Name of the system to create
		 * @param  path Path associated with the system
		 * @return      The instance of the system or nullptr if the system is not registred.
		 */
		static
		system_ptr_t create_system(const std::string& name, const std::string& path="");

		/**
		 * Register a system to the factory. (Preferably, use macro PRX_REGISTER_SYSTEM instead of this function)
		 * @param  name Name of the system. Note that one can register multiple versions of the same system by assigning different names
		 * @param  func The constructor to be used to generate the system
		 * @return      True if the registration was successful. False otherwise.
		 */
		bool register_system(const std::string name, system_gen_fn func);

		/**
		 * Register a function to compute a maximum velocity of a system. Note that the factory 
		 * can store multiple functions for the same system as long as name changes. 
		 * I.e. for an omni system one could have one function per dimension
		 * @param  name Name to associated the function with
		 * @param  func Function to call when querying
		 * @return      True if the registration was successful. False otherwise.
		 */
		bool register_velocity_fn(const std::string name, velocity_gen_fn func);

		/**
		 * Get the max velocity for the given system
		 * @param  name  Name of the function to look for
		 * @param  system system to calculate the max velocity
		 * @return       The max velocity. Infinity if something failed along the way.
		 */
		static double get_system_max_velocity(const std::string& name, const system_ptr_t system);

		/**
		 * Returns the names of the available systems
		 */
    	static std::vector<std::string> available_systems();

    	/**
    	 * Returns the names of the available functions
    	 */
    	static std::vector<std::string> available_velocity_functions();

	private:
    	std::unordered_map<std::string, system_gen_fn> system_generators;
    	std::unordered_map<std::string, velocity_gen_fn> max_vel_generators;

		system_factory_t();
		
	};

}
/**
 * Macro to registrer a system to the factory
 * @param  SYSTEM_CLASS The class to registrer
 * @param  SYSTEM_NAME  The name to associate to this system
 * @return              If the registration is successful, the variable prx::factory_registration::VAR_##SYSTEM_NAME##_REGISTRED is true.
 */
#define PRX_REGISTER_SYSTEM(SYSTEM_CLASS, SYSTEM_NAME) \
namespace prx { namespace factory_registration \
{ \
	static auto FN_##SYSTEM_NAME##_GENERATOR = [](std::string path) \
	{ \
		system_ptr_t new_ptr; \
		new_ptr.reset(new SYSTEM_CLASS(path)); \
		return new_ptr; \
	}; \
	const bool VAR_##SYSTEM_NAME##_REGISTRED = system_factory_t::get().register_system(#SYSTEM_NAME, FN_##SYSTEM_NAME##_GENERATOR); \
} }

// TODO: Test this macro. A change to sys_gen_fn might be needed to accept variadic args.
/**
 * Macro to registrer a system to the factory using variadic arguments. Not tested yet.
 * @param  SYSTEM_CLASS The class to registrer
 * @param  SYSTEM_NAME  The name to associate to this system
 * @return              If the registration is successful, the variable prx::factory_registration::VAR_##SYSTEM_NAME##_REGISTRED is true.
 */
#define PRX_REGISTER_CUSTOM_SYSTEM(SYSTEM_CLASS, SYSTEM_NAME, ...) \
namespace prx { namespace factory_registration \
{ \
	static auto FN_##SYSTEM_NAME##_GENERATOR = [](std::string path, __VA_ARGS__) \
	{ \
		system_ptr_t new_ptr; \
		new_ptr.reset(new SYSTEM_CLASS(path, __VA_ARGS__)); \
		return new_ptr; \
	}; \
	const bool VAR_##SYSTEM_NAME##_REGISTRED = system_factory_t::get().register_system(#SYSTEM_NAME, FN_##SYSTEM_NAME##_GENERATOR); \
} }

/**
 * Registrer a function to compute the max velocity to the factory
 * @param  SYSTEM_NAME Name to be associated to this function
 * @param  VEL_FN      The function to registrer
 * @return             If the registration is successful, the variable prx::factory_registration::VAR_##SYSTEM_NAME##_VEL_FN_REGISTRED is true.
 */
#define PRX_REGISTER_VELOCITY_FN(SYSTEM_NAME, VEL_FN) \
namespace prx { namespace factory_registration \
{ \
	const bool VAR_##SYSTEM_NAME##_VEL_FN_REGISTRED = system_factory_t::get().register_velocity_fn(#SYSTEM_NAME, VEL_FN); \
} }



