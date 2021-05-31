#include "prx/utilities/general/param_loader.hpp"
#include "prx/utilities/general/string_manip.hpp"
#include "prx/utilities/general/constants.hpp"

namespace prx
{

	param_loader::param_loader()
	{
		set_input_path(input_path);
	}
	param_loader::param_loader(std::string file_name)
	{
		set_input_path(input_path);
		add_file(file_name);
	}

	param_loader::param_loader(int argc, char* argv[])
	{
		set_input_path(input_path);
		add_opts(argc, argv);
	}

	param_loader::param_loader(std::string file_name, int argc, char* argv[])
	{
		set_input_path(input_path);
		add_file(file_name);
		add_opts(argc, argv);
	}

	param_loader::param_loader(const param_loader& pl)
	{
		set_input_path(input_path);
		params = std::move(pl.params);
	}

	param_loader::param_loader(YAML::Node input_params)
	{
		params = std::move(input_params);
	}

	void param_loader::add(const param_loader& pl)
	{
		params.push_back(std::move(pl.params));
	}

	void param_loader::add_file(std::string file_name)
	{
		try
		{
			auto nn = YAML::LoadFile(pl_input_path+file_name);
			params = std::move(expand_file(nn));
		}
		catch(...)
		{
			if(params.IsNull())
				prx_throw("Bad filename to param_loader '"<<pl_input_path<<file_name<<"'");
		}
		
	}

	YAML::Node param_loader::expand_file(YAML::Node& node)
	{
		YAML::Node new_node;
		// Do nothing if node has nothing
		if (!node.IsDefined()) return new_node;
		
		if ( node.Tag() == "!file" )
		{
			new_node = param_loader(node.as<std::string>()).params;
		}
		else
		{
			for (auto p : node) 
			{
				auto expanded = expand_file(p.second);
				if (!expanded.IsNull())
				{
					node[p.first.as<std::string>()] = std::move(expanded);
				}
			}
			new_node = std::move(node);
		}
		return new_node;
	}

	YAML::Node param_loader::find(const std::string& key, YAML::Node node)
	{
		YAML::Node n;
		for (auto p : node)
		{
			if (p.first.as<std::string>() == key)
			{
				return std::move(node[p.first.as<std::string>()]);
			}
			if (p.second.IsMap())
			{
				n = std::move(find(key, p.second));
			}
		}
		// return a null node if nothing was found
		return std::move(n);
	}

	void param_loader::add_opts(int argc, char* argv[])
	{
		const std::regex opt_regex_norm("--\\w+=[\\.\\w]+");
		const std::regex opt_regex_bool("--\\w+");
		const std::regex opt_regex_file("--\\w+=!file ([\\.\\w]+/?)+.yaml");
		const std::regex opt_regex_vect("--\\w+=\\[(.)+\\]");

		YAML::Node n;

		// i = 0 is the executable... usually
		// Should we save the executable's name?
    	for (int i = 0; i < argc; ++i)
    	{
    	    std::string opt(argv[i]);
    	        std::cout << opt << std::endl;
    	    if (std::regex_match(opt, opt_regex_norm) )
    	    {
    	        auto pos = opt.find("=");
    	        std::cout << opt << std::endl;
    	        n[opt.substr(2,pos-2)] = opt.substr(pos+1);

    	    }
    	    else if (std::regex_match(opt, opt_regex_bool))
    	    {
    	        auto p = find(opt.substr(2), params);
    	        // if (p.IsNull())
    	        // {
    	        	n[opt.substr(2)] = "true";
    	        // }
    	        // else
    	        // {
    	        // 	p = "true";
    	        // }
    	    }
    	    else if (std::regex_match(opt, opt_regex_file))
    	    {
    	    	YAML::Node file_n;

    	    	auto pos_eq = opt.find("=");
    	    	auto pos_fi = opt.find("!file");
    	     //    auto p = find(opt.substr(2,pos-2), params);
    	    	// if (p.IsNull())
    	     //    {
    	        	file_n = opt.substr(pos_fi+6);
    	    	file_n.SetTag("!file");
    	        // }
    	        // else
    	        // {
    	        	// p = opt.substr(pos+1);
    	        // }
				n[opt.substr(2,pos_eq-2)] = expand_file(file_n);
    	        // 
    	    }
    	    else if (std::regex_match(opt, opt_regex_vect))
    	    {
    	    	auto pos = opt.find("=");
    	        std::cout << opt << std::endl;
    	        n[opt.substr(2,pos-2)] = YAML::Load(opt.substr(pos+1));
    	    }
    	    else
    	    {
    	    	prx_warn("Error adding: " << opt.substr(2) );
    	    }
    	}
		// std::cout << "-------" << std::endl;
    	// print(n);
    	// params = std::move(expand_file(n));
		this -> merge(std::move(n));
		// std::cout << "-------" << std::endl;
    	// print(params);
		// std::cout << "-------" << std::endl;

	}

	void param_loader::merge(const YAML::Node& other)
	{
		for (auto p : other) 
		{

			params[p.first.as<std::string>()] = std::move(p.second);
			
		}
	}

	const param_loader param_loader::operator[](const std::string& key) const
	{
		std::string::size_type subkey_init = key[0] == '/' ? 1 : 0;
		std::string::size_type subkey_pos = 0;// = key.find("/", subkey_init);

		subkey_pos = key.find("/", subkey_init);
		auto new_node = params[key.substr(subkey_init, subkey_pos - subkey_init)];
		if (subkey_pos == std::string::npos)
		{
			if(!new_node.IsDefined())
			{
				prx_throw("Tried to access element \""<<key<<"\" which isn't there.");
			}
	
			return param_loader(new_node);
		}
		// Tail recursive!
		return param_loader(new_node)[key.substr(subkey_pos)];
		

	}

	param_loader param_loader::operator[](const std::string& key)
	{
		std::string::size_type subkey_init = key[0] == '/' ? 1 : 0;
		std::string::size_type subkey_pos = 0;// = key.find("/", subkey_init);
		// YAML::Node new_node = params;

		// std::cout << "key: " << key << std::endl;
		subkey_pos = key.find("/", subkey_init);
		// std::cout << "\tsubkey: " << key.substr(subkey_init, subkey_pos - subkey_init) << " init: " << subkey_init << " pos: " << subkey_pos << std::endl;
		auto new_node = params[key.substr(subkey_init, subkey_pos - subkey_init)];
		if (subkey_pos == std::string::npos)
		{	
			return param_loader(new_node);
		}
		else
		{
			// Tail recursive!
			return param_loader(new_node)[key.substr(subkey_pos)];
		}
		// while (subkey_pos != std::string::npos)
		// {
			// subkey_init++;
			// subkey_pos = key.find("/", subkey_init);
			// std::cout << "\tsubkey: " << key.substr(subkey_init, subkey_pos - subkey_init) << " init: " << subkey_init << " pos: " << subkey_pos << std::endl;
			// new_node = new_node[key.substr(subkey_init, subkey_pos - subkey_init)];
			// subkey_init = subkey_pos + 1;
		// }
	}

	void param_loader::print()
	{
	    print(params);
	}

	void param_loader::print(const YAML::Node& pl, std::string prepath)
	{
		bool nf = false;
		switch (pl.Type())
		{
			case YAML::NodeType::Undefined : printf(": Undefined\n");
				break;
			case YAML::NodeType::Null : printf("%s\n", prepath.c_str());
				break;
			case YAML::NodeType::Scalar : printf("%s: %s\n", prepath.c_str(), pl.as<std::string>().c_str());
				break;
			case YAML::NodeType::Sequence : 
				printf("%s: [", prepath.c_str());
				for (auto p : pl)
				{
					if (nf) printf(",");
					printf(" %s", p.as<std::string>().c_str());
					nf = true;
				}
				printf(" ]\n");
				break;
			case YAML::NodeType::Map : 
				for (auto p : pl) 
				{
					// printf("/%s", p.first.as<std::string>().c_str());
					// printf("tag: %s\n", p.second.Tag().c_str());
					print(p.second, prepath + "/" + p.first.as<std::string>());
				}
				break;
			default: prx_throw("Problem printing param_loader! Is there a new type?\n");
		}
		
	}
// 	template<class T>
// std::vector<T> str_to_vec(std::string str, char sep = ',')
// {   
//     std::vector<T> v;
//     int pre = 0;
//     int pos = str.find(sep);

//     T value;
//     while(pre != std::string::npos)
//     {
//         std::stringstream convert(str.substr(pre, pos - pre));
//         convert >> value;
//         v.push_back(value);
//         pre = pos + (pos == std::string::npos ? 0 : 1);
//         pos = str.find(sep, pre);
//     }
//     return v;
// }
}