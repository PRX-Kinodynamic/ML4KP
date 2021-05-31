#include "prx/simulation/system_factory.hpp"

namespace prx
{
	using T_create_system = std::function<system_ptr_t()>;

    system_factory_t::system_factory_t()
    {

    }

    system_factory_t& system_factory_t::get()
    {
        static system_factory_t instance;
        return instance;
    }
    
	bool system_factory_t::register_system(const std::string name, system_gen_fn func)
	{
        bool res;
        auto it = system_factory_t::get().system_generators.find(name);
        if (it != system_factory_t::get().system_generators.end())
        {
            res = true;
        }
        else
        {
            res = system_generators.insert(std::make_pair(name, func)).second;
            prx_assert(res, "Problem registering system: " << name);
        }
        return res;
	}

    bool system_factory_t::register_velocity_fn(const std::string name, velocity_gen_fn func)
    {
        bool res;
        auto it = system_factory_t::get().max_vel_generators.find(name);
        if (it != system_factory_t::get().max_vel_generators.end())
        {
            res = true;
        }
        else
        {
            res = max_vel_generators.insert(std::make_pair(name, func)).second;
            prx_assert(res, "Problem registering max velocity for: " << name);
        }
        return res;
    }

	system_ptr_t system_factory_t::create_system(const std::string& name, const std::string& path)
    {
    	auto it = system_factory_t::get().system_generators.find(name);
        if (it != system_factory_t::get().system_generators.end())
        {
            return it->second(path);
        }
        
        return nullptr;
    };

    std::vector<std::string> system_factory_t::available_velocity_functions()
    {
        std::vector<std::string> r;
        for (auto it : system_factory_t::get().max_vel_generators)
        {
            r.push_back(it.first);
        }
        return r;
    }

    std::vector<std::string> system_factory_t::available_systems()
    {
        std::vector<std::string> r;
        for (auto it : system_factory_t::get().system_generators)
        {
            r.push_back(it.first);
        }
        return r;
    }

    double system_factory_t::get_system_max_velocity(const std::string& name, const system_ptr_t system)
    {
        auto it = system_factory_t::get().max_vel_generators.find(name);
        if (it != system_factory_t::get().max_vel_generators.end())
        {
            return it->second(system);
        }
        prx_warn("No function associated to '" << name << "' found! Returning inf.")
        return std::numeric_limits<double>::infinity();
    }

}