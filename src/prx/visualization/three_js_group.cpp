#include "prx/visualization/three_js_group.hpp"
#include <fstream>

namespace prx
{
	void three_js_group_t::vis_info_t::update_info()
	{
		auto config_ptr = transform.lock();
		rotation = quaternion_t(config_ptr->linear());
		position = config_ptr->translation();
	}

	three_js_group_t::three_js_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles)
	{
		state_infos.clear();
		obstacle_infos.clear();

		for(auto&& object: in_obstacles)
		{
			auto geoms = object->get_geometries();
			auto configs = object->get_configurations();
			for(int i=0;i<geoms.size();i++)
			{
				prx_assert(geoms[i].first==configs[i].first,"Geometry and configuration lists don't match in "<<object->get_object_name());
				auto g = geoms[i].second;
				auto config = configs[i].second;
				auto g_ptr = g.lock();
				auto info = std::make_shared<vis_info_t>();
				info->geom_type = g_ptr->get_geometry_type();
				info->geom_params = g_ptr->get_geometry_params();
				info->transform = config;
				info->body_name = geoms[i].first;
				info->color = g_ptr->get_visualization_color();
				info->update_info();
				obstacle_infos.push_back(info);
			}
		}

		for(auto&& sys: in_plants)
		{
			auto plant = std::dynamic_pointer_cast<plant_t>(sys);
			if(plant)
			{
				plants.push_back(sys);
				auto geoms = plant->get_geometries();
				auto configs = plant->get_configurations();
				for(int i=0;i<geoms.size();i++)
				{
					prx_assert(geoms[i].first==configs[i].first,"Geometry and configuration lists don't match in "<<plant->get_object_name());
					auto g = geoms[i].second;
					auto config = configs[i].second;
					auto g_ptr = g.lock();
					auto info = std::make_shared<vis_info_t>();
					info->geom_type = g_ptr->get_geometry_type();
					info->geom_params = g_ptr->get_geometry_params();
					info->transform = config;
					info->body_name = geoms[i].first;
					info->color = g_ptr->get_visualization_color();
					state_infos.push_back(info);
				}
			}
		}
	}

	three_js_group_t::~three_js_group_t()
	{
	}


	void three_js_group_t::update_vis_infos()
	{
		update_plants();
	}

	void three_js_group_t::add_vis_infos(info_geometry_t info_type, std::vector<trajectory_t>& tree_vis, std::string body_name, space_t* state_space, std::string color)
	{
		for(auto& traj : tree_vis)
        {
            add_vis_infos(info_type, traj, body_name, state_space, color);
        }
	}


	void three_js_group_t::add_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name, space_t* state_space, std::string color)
	{
		std::shared_ptr<vis_info_t> selected_info;
		for(auto&& element: state_infos)
		{
			if(element->body_name ==body_name)
			{
				selected_info = element;
				break;
			}
		}
		prx_assert(selected_info,"Tried to generate extra info geometries with rigid body name "<<body_name<<", but that geometry doesn't exist.");
		if (traj.size() == 0)
		{
			prx_warn("Trajectory of 0 size!");
			return;
		}
		if(info_type == info_geometry_t::LINE)
		{
			std::vector<vector_t> positions;

			for(double i=0;i<=1.0;i+=.1)
			{
				auto s = traj.at(i);
				state_space->copy_from_point(s);
				update_plants();
				positions.push_back(selected_info->position);
			}

			add_vis_infos(info_type,std::move(positions),color);
		}
		else if(info_type == info_geometry_t::FULL_LINE)
		{
			std::vector<vector_t> positions;

			for(double i=0;i<=1.0;i+=.01)
			{
				auto s = traj.at(i);
				state_space->copy_from_point(s);
				update_plants();
				positions.push_back(selected_info->position);
			}

			add_vis_infos(info_type,std::move(positions),color);
		}
		else
		{
			prx_throw("Trajectory visualization was provided the wrong info type...");
		}
	}

	void three_js_group_t::add_vis_infos(info_geometry_t info_type, std::vector<vector_t> positions, std::string color, double radius )
	{
		info_geoms.push_back({info_type,std::move(positions),color,radius});
	}

	void three_js_group_t::add_detailed_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name, space_t* state_space, std::string color)
	{
		std::shared_ptr<vis_info_t> selected_info;
		for(auto&& element: state_infos)
		{
			if(element->body_name ==body_name)
			{
				selected_info = element;
				break;
			}
		}
		prx_assert(selected_info,"Tried to generate extra info geometries with rigid body name "<<body_name<<", but that geometry doesn't exist.");

		if (traj.size() == 0)
		{
			prx_warn("Trajectory of 0 size!");
			return;
		}
		double step;
		if(info_type == info_geometry_t::LINE)
		{
			step = 0.1;
		}
		else if(info_type == info_geometry_t::FULL_LINE)
		{
			step = 0.01;
		}
		else
		{
			prx_throw("Trajectory visualization was provided the wrong info type...");
		}

		auto pt = state_space->make_point();
		const auto traj_section = [&](space_point_t pt1, space_point_t pt2)
		{	
			std::vector<vector_t> positions;
		
			for(double i=0;i<=1.0;i+=step)
			{
				state_space->interpolate(pt1, pt2, i, pt);	
				state_space->copy_from_point(pt);
				this -> update_plants();
				positions.push_back(selected_info->position);
			}
			
			return positions;
		};

		for (auto it=traj.begin(); it != traj.end() - 1; ++it )
		{
			// std::vector<vector_t> positions = 
			add_vis_infos(info_type,std::move(traj_section(*it, *(it + 1))),color);
		}

	}

	void three_js_group_t::add_animation(const trajectory_t& traj, space_t* state_space, space_point_t default_state)
	{
        double timestamp=0;
        prx_warn_cond(traj.size()>0, "Empty trajectory!");
        if(traj.size()==0 && default_state != nullptr)
        {
            state_space -> copy_from_point(default_state);
            snapshot_state(timestamp);
            timestamp+=simulation_step;
        }
        else
        {
			for(auto state : traj)
        	{
	           	state_space -> copy_from_point(state);
	           	snapshot_state(timestamp);
	            timestamp+=simulation_step;
        	}
        }
	}

	void three_js_group_t::update_plants()
	{
		for(auto&& sys: plants)
		{
			auto plant = std::dynamic_pointer_cast<plant_t>(sys);
			plant->update_configuration();
		}

		for(auto&& element: state_infos)
		{
			element->update_info();
		}
	}

	void three_js_group_t::add_tree_log(std::string log_name, space_t* state_space)
	{
		tree_log_file = log_name;
		state_space_dim = state_space -> get_dimension();
		
	}

	void three_js_group_t::snapshot_state(double timestamp)
	{
		update_vis_infos();
		std::vector<double> params;
		for(auto&& element: state_infos)
		{
			std::vector<double> params_internal = {element->position.x(),element->position.y(),element->position.z(),
						element->rotation.x(),element->rotation.y(),element->rotation.z(),element->rotation.w()};
			params.insert(std::end(params), std::begin(params_internal), std::end(params_internal));
		}
		params.push_back(timestamp);
		plant_animation_params.push_back(params);
	}

	void three_js_group_t::output_html(std::string filename)
	{
		const auto vis_geometry_init = [](std::shared_ptr<vis_info_t>& element,std::string& js_string)
		{
			if(element->geom_type==geometry_type_t::BOX)
			{
				js_string+="var geometry = new THREE.BoxGeometry( "+std::to_string(element->geom_params[0])+", "
																   +std::to_string(element->geom_params[1])+", "
																   +std::to_string(element->geom_params[2])+" );";
			}
			else if(element->geom_type==geometry_type_t::CYLINDER)
			{
				js_string+="var geometry = new THREE.CylinderGeometry( "+std::to_string(element->geom_params[0])+", "
																		+std::to_string(element->geom_params[0])+", "
																		+std::to_string(element->geom_params[1])+", 32 );";
				js_string+="geometry.applyMatrix( new THREE.Matrix4().makeRotationX(Math.PI/2) );";

			}
			else if(element->geom_type==geometry_type_t::CAPSULE)
			{
				js_string+="var geometry = new CapsuleBufferGeometry( "+std::to_string(element->geom_params[0])+", "
																		+std::to_string(element->geom_params[0])+", "
																		+std::to_string(element->geom_params[1])+", 32 );";
				js_string+="geometry.applyMatrix( new THREE.Matrix4().makeRotationX(Math.PI/2) );";
			}
			else if(element->geom_type==geometry_type_t::ELLIPSOID)
			{
				js_string+="var geometry = new THREE.SphereGeometry( 1, 32, 32 );";
				js_string+="geometry.applyMatrix( new THREE.Matrix4().makeScale( "+std::to_string(element->geom_params[0])+", "
																				  +std::to_string(element->geom_params[1])+", "
																				  +std::to_string(element->geom_params[2])+" ) );";
			}
			else if(element->geom_type==geometry_type_t::SPHERE)
			{
				js_string+="var geometry = new THREE.SphereGeometry( "+std::to_string(element->geom_params[0])+", 32, 32 );";
			}
			else
			{
				prx_throw("Unknown geometry type in three_js_group");
			}
		};


		std::string full_filename = lib_path+"out/"+filename;
		std::cout<<"Outputting visualization at "<<full_filename<<std::endl;
		std::ofstream fout;
		fout.open(full_filename.c_str());
		fout<<html_header;

		//visualize obstacles
		for(auto&& element: obstacle_infos)
		{
			std::string js_string;
			vis_geometry_init(element,js_string);
			js_string+="var material = new THREE.MeshPhongMaterial( { color: "+element->color+", flatShading: true } );";
			js_string+="var mesh = new THREE.Mesh( geometry, material );";
			js_string+="mesh.position.set( "+std::to_string(element->position.x())+", "+std::to_string(element->position.y())+", "+std::to_string(element->position.z())+" );";
			js_string+="mesh.setRotationFromQuaternion(new THREE.Quaternion("+std::to_string(element->rotation.x())+","
																		+std::to_string(element->rotation.y())+","
																		+std::to_string(element->rotation.z())+","
																		+std::to_string(element->rotation.w())+"));";
			js_string+="mesh.castShadow=true;";
			js_string+="mesh.receiveShadow=true;";
			js_string+="scene.add( mesh );\n";
			fout<<js_string;
		}

		//visualize info geometries
		for(auto&& element : info_geoms)
		{
			std::string js_string;
			if(element.first==info_geometry_t::LINE && element.second.size()>=2)
			{
				js_string+="var geometry = new THREE.BufferGeometry();";
				js_string+="var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
				js_string+="var positions = [];";
				js_string+="var colors = [];";

				auto color_vals = string_to_rgb(element.third);

				for(int i=0;i<element.second.size();i++)
				{
					js_string+="positions.push( "+std::to_string(element.second[i].x())+", "+std::to_string(element.second[i].y())+", "+std::to_string(element.second[i].z())+" );";
					js_string+="colors.push("+std::to_string(color_vals.x())+","+std::to_string(color_vals.y())+","+std::to_string(color_vals.z())+");";
				}
				js_string+="geometry.addAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
				js_string+="geometry.addAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );";
				js_string+="geometry.computeBoundingSphere();";
				js_string+="mesh = new THREE.Line( geometry, material );";
				js_string+="scene.add( mesh );\n";
			}
			else if(element.first==info_geometry_t::FULL_LINE && element.second.size()>=2)
			{
				js_string+="var geometry = new THREE.BufferGeometry();";
				js_string+="var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
				js_string+="var positions = [];";
				js_string+="var colors = [];";

				auto color_vals = string_to_rgb(element.third);

				for(int i=0;i<element.second.size();i++)
				{
					js_string+="positions.push( "+std::to_string(element.second[i].x())+", "+std::to_string(element.second[i].y())+", "+std::to_string(element.second[i].z()+0.05)+" );";
					js_string+="colors.push("+std::to_string(color_vals.x())+","+std::to_string(color_vals.y())+","+std::to_string(color_vals.z())+");";
				}
				js_string+="geometry.addAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
				js_string+="geometry.addAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );";
				js_string+="geometry.computeBoundingSphere();";
				js_string+="mesh = new THREE.Line( geometry, material );";
				js_string+="scene.add( mesh );\n";
			}
			else if(element.first==info_geometry_t::QUAD && element.second.size()>=4)
			{
				js_string+="var geometry = new THREE.Geometry();";
				js_string+="var material = new THREE.MeshPhongMaterial( { color: "+element.third+" , side: THREE.DoubleSide, flatShading: true } );";

				for(int i=0;i<element.second.size();i++)
				{
					js_string+="geometry.vertices.push( new THREE.Vector3("+std::to_string(element.second[i].x())+", "+std::to_string(element.second[i].y())+", "+std::to_string(element.second[i].z())+") );";
				}
				js_string+="geometry.faces.push(new THREE.Face3(0, 1, 2));";
				js_string+="geometry.faces.push(new THREE.Face3(0, 3, 2));";
				js_string+="mesh = new THREE.Mesh( geometry, material );";
				js_string+="scene.add( mesh );\n";
			}
			else if (element.first == info_geometry_t::CIRCLE && element.second.size() >=1)
			{	// TODO: FIX THIS
				// js_string+="var geometry = new THREE.CircleBufferGeometry( 1, 32 );";
				// js_string+="var geometry = new THREE.CircleBufferGeometry( 1, 32 );";
				// js_string+="var material = new THREE.MeshBasicMaterial( { color: 0xffff00 } );";
				// // js_string+="var material = new THREE.MeshBasicMaterial( { color: " + element.third + "} );";
				// js_string+="var positions = [];";

				// // auto color_vals = string_to_rgb(element.third);

				// // for(int i=0;i<element.second.size();i++)
				// // {
				// 	js_string+="positions.push( 0,0, 0.1 );";
				// 	// js_string+="positions.push( "+std::to_string(element.second[0].x())+", "+std::to_string(element.second[0].y())+", 0.1 );";
				// 	// js_string+="colors.push("+std::to_string(color_vals.x())+","+std::to_string(color_vals.y())+","+std::to_string(color_vals.z())+");";
				// // }
				// js_string+="geometry.setAttribute( 'position', new THREE.BufferAttribute( positions, 3 ) );";
				// // js_string+="geometry.addAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );";
				// // js_string+="geometry.computeBoundingSphere();";
				// js_string+="var circle = new THREE.Mesh( geometry, material );";
				// js_string+="var geometry = new THREE.CircleBufferGeometry( 1, 32 );";
				// js_string+="var vertices = new Float32Array( [ 30, 30, 0 ] );";

				// js_string+="geometry.setAttribute( 'position', new THREE.BufferAttribute( vertices, 3 ) );";
				// js_string+="var material = new THREE.MeshBasicMaterial( { color: 0xff0000 } );";
				// js_string+="var circle = new THREE.Mesh( geometry, material );";
				// js_string+="scene.add( circle );\n";
				// std::cout << js_string;
			}
			fout<<js_string;
		}

		// Growth of tree animation
		int dim = state_space_dim>3?state_space_dim:3;
		const auto find_coords = [dim](std::string line)
		{
			std::vector<n_vector_t<Eigen::Dynamic>> coords;
			n_vector_t<Eigen::Dynamic> vec;
			// Eigen::VectorXd vec(Eigen::Dynamic);
			std::string::size_type n = line.find('T');
			int i = 0;

			std::istringstream ss(line.substr(n+2));
			std::string token, num;
			while(std::getline(ss, token, 'T')) 
			{
				std::istringstream ss_c(token);
			    // std::cout << token << '\n';
				vec.resize(dim);
				vec = Eigen::MatrixXd::Zero(dim, 1);
				i = 0;
				while(std::getline(ss_c, num, ',')) 
				{
			    	// std::cout << num << '\n';
					if (num.size() > 0)
					{
						vec[i] = std::stof(num);
						i++;
					}
				}
				coords.push_back(std::move(vec));
			}

			return coords;
		};
		std::string max_time = "0.0";
		if (tree_log_file.size() > 0)
		{
			std::map<long unsigned int, std::vector<std::pair<char, double> > > actions_time;
			std::map<long unsigned int, std::string > trajs;
			std::vector<std::string> grid_pts;
			std::ifstream input(tree_log_file);
			long unsigned int index = 0;
			double time = 0.0;
			std::string::size_type n, nn;
			for( std::string line; getline( input, line ); )
			{
				if (line.size() == 0) continue;
				if(line[0] == 'G')
				{
					grid_pts.push_back(line);
					continue;
				}
				n = line.substr(2).find(',');
				time = stod(line.substr(2,n));
				if (line[0] == 'E')
				{
					max_time = line.substr(2,n);
					continue;
				}

				n = line.find('i');
				nn = line.substr(n+2).find(',');
				index = std::stoi(line.substr(n+2, nn));
				// std::cout << line << std::endl;
				// std::cout << "index: " << index << std::endl;
				if (trajs.find(index) == trajs.end())
				{
					trajs.emplace(index, line);
					actions_time.emplace(index, std::vector < std::pair < char, double > > ());
				}
				// trajs[index] = line;
				actions_time[index].push_back(std::make_pair(line[0], time));
			}
			std::string black = "0x000000";
			for (auto t : trajs)
			{
				std::string js_string;
				// std::cout << "index: " << t.first << std::endl;

				// for (int i = 0; i < actions_time[t.first].size(); ++i)
				// {
				// 	std::cout << "\tAction: " << actions_time[t.first][i].first << "\ttime: "
				// 		<< actions_time[t.first][i].second << "\n";
				// 	/* code */
				// }
				
				js_string+="var geometry = new THREE.BufferGeometry();";
				js_string+="var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
				js_string+="var positions = [];";
				js_string+="var colors = [];";
				
				auto coords = find_coords(t.second);

				for (int i=0; i < coords.size();i++)
				{	
					// std::cout << "\t" << coords[i][0] << "," << coords[i][1] << std::endl;
					js_string+="positions.push( "+std::to_string(coords[i][0])+", "+std::to_string(coords[i][1])+", "+std::to_string(coords[i][2]+0.05)+" );";
					js_string+="colors.push("+black+","+black+","+black+");";
					// js_string+="colors.push("+std::to_string(color_vals.x())+","+std::to_string(color_vals.y())+","+std::to_string(color_vals.z())+");";


				}
				js_string+="geometry.addAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
				js_string+="geometry.addAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );";
				js_string+="geometry.computeBoundingSphere();";
				js_string+="mesh = new THREE.Line( geometry, material );";
				js_string+="scene.add( mesh );\n";

				js_string+="var visibility_tree = new THREE.BooleanKeyframeTrack( '.visible', [0.0";

				for (int i = 0; i < actions_time[t.first].size(); ++i)
				{

					js_string+=",";
					js_string+=std::to_string(actions_time[t.first][i].second);
					
				}
				js_string+=",";
				js_string+=max_time;
				js_string+="],[false";
				std::string bool_val = "true";
				for (int i = 0; i < actions_time[t.first].size(); ++i)
				{

					switch (actions_time[t.first][i].first)
					{
						case 'A':
							bool_val="true";
							break;
						case 'D':
							bool_val="false";
							break;
					}
					js_string+=",";
					js_string+=bool_val;
				}
				js_string+=",";
				js_string+=bool_val;
				js_string+="]);";

				js_string+="var clip = new THREE.AnimationClip( 'Tree_growth', -1, [visibility_tree] );";
				js_string+="var mixer = new THREE.AnimationMixer( mesh );";
				js_string+="var clipAction = mixer.clipAction( clip );";
				js_string+="clipAction.play();";
				js_string+="mixers.push(mixer);\n";

				// std::cout << js_string;
				fout<<js_string;
			}

			// std::string gray = "0xb8b8b8";
			std::string gray = "0xeb4034";

			std::string num;
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			int j = 0;
			for (auto pt : grid_pts)
			{

				std::istringstream ss_c(pt);
				while(std::getline(ss_c, num, ','))
				{
					if (j == 2)
					{
					// std::cout << "x: " << num;
						x = std::stod(num);
					}
					if (j == 3)
					{
					// std::cout << "\ty: " << num << std::endl;
						y = std::stod(num);
						j = 0;
					}
					else
					{
						j++;
					}
				}
				std::string js_string;
				js_string+="var geometry = new THREE.BufferGeometry();";
				js_string+="var material = new THREE.LineBasicMaterial( {color:"+gray+ ", linewidth: 2} );";
				// js_string+="var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
				js_string+="var positions = [];";
				// for(int i=0;i<2;i++)
				// {
				js_string+="positions.push( "+std::to_string(x - 1)+", "+std::to_string(y)+", "+std::to_string(z + 0.01)+" );";
				js_string+="positions.push( "+std::to_string(x + 1)+", "+std::to_string(y)+", "+std::to_string(z + 0.01)+" );";
					
				// }
				js_string+="geometry.addAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
				js_string+="geometry.computeBoundingSphere();";
				js_string+="mesh = new THREE.Line( geometry, material );";
				js_string+="scene.add( mesh );\n";


				js_string+="var geometry = new THREE.BufferGeometry();";
				js_string+="var material = new THREE.LineBasicMaterial( {color:"+gray+ ", linewidth: 2} );";
				// js_string+="var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
				js_string+="var positions = [];";

				js_string+="positions.push( "+std::to_string(x)+", "+std::to_string(y - 1)+", "+std::to_string(z + 0.01)+" );";
				js_string+="positions.push( "+std::to_string(x)+", "+std::to_string(y + 1)+", "+std::to_string(z + 0.01)+" );";

				js_string+="geometry.addAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
				js_string+="geometry.computeBoundingSphere();";
				js_string+="mesh = new THREE.Line( geometry, material );";
				js_string+="scene.add( mesh );\n";

				fout<<js_string;
			}
			// std::ifstream input(tree_log_file);
			// // std::vector<Eigen::VectorXd> coords_parent;
			// std::vector<n_vector_t<Eigen::Dynamic>> coords;
			// std::vector<double> coords_child;

			// for( std::string line; getline( input, line ); )
			// {
			// 	std::cout << line << std::endl;
			// 	switch(line[0])
			// 	{
			// 		case 'A': // Addition of a node 
			// 			coords = find_coords(line);
			// 			// for (int i = 0; i < coords.size(); ++i)
			// 			// {
			// 			// 	std::cout << "coords[" << i << "]:\t" << coords[i][0] << "," << coords[i][1] << std::endl;
			// 			// }
			// 			js_string+="var geometry = new THREE.BufferGeometry();";
			// 			js_string+="var material = new THREE.LineBasicMaterial( { vertexColors: THREE.VertexColors } );";
			// 			js_string+="var positions = [];";
			// 			js_string+="var colors = [];";

			// 			auto color_vals = string_to_rgb(element.third);

			// 			for(int i=0;i<cords.size();i++)
			// 			{
			// 				js_string+="positions.push( "+std::to_string()+", "+std::to_string(element.second[i].y())+", "+std::to_string(element.second[i].z())+" );";
			// 				js_string+="colors.push("+std::to_string(color_vals.x())+","+std::to_string(color_vals.y())+","+std::to_string(color_vals.z())+");";
			// 			}
			// 			js_string+="geometry.addAttribute( 'position', new THREE.Float32BufferAttribute( positions, 3 ) );";
			// 			js_string+="geometry.addAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );";
			// 			js_string+="geometry.computeBoundingSphere();";
			// 			js_string+="mesh = new THREE.Line( geometry, material );";
			// 			js_string+="scene.add( mesh );\n";
			// 			break;
			// 		case 'D': // Deletion of a node
			// 			break;
			// 	}
			//     // find_coords(line, 'P');
			// }
		}


		// Plant animation
		unsigned body_index=0;
		for(auto&& element: state_infos)//plant geometries iterator
		{
			if(plant_animation_params.size()==0)
			{
				continue;
			}
			std::string js_string;

			vis_geometry_init(element,js_string);

			js_string+="var material = new THREE.MeshPhongMaterial( { color: "+element->color+", flatShading: true } );";
			js_string+="var mesh = new THREE.Mesh( geometry, material );";
			js_string+="mesh.castShadow=true;";
			js_string+="mesh.receiveShadow=true;";
			js_string+="scene.add( mesh );";

			js_string+="var positionKF = new THREE.VectorKeyframeTrack( '.position', [";
			//timestamp reading
			for(int i=0;i<plant_animation_params.size();i++)
			{
				if(i!=0)
				{
					js_string+=",";
				}
				auto& time_stamp_params = plant_animation_params[i];
				js_string+=std::to_string(time_stamp_params.back());
			}
			js_string+="],[";

			//position reading
			for(int i=0;i<plant_animation_params.size();i++)
			{
				if(i!=0)
				{
					js_string+=",";
				}
				auto& time_stamp_params = plant_animation_params[i];
				js_string+=std::to_string(time_stamp_params[body_index*7+0]);
				js_string+=",";
				js_string+=std::to_string(time_stamp_params[body_index*7+1]);
				js_string+=",";
				js_string+=std::to_string(time_stamp_params[body_index*7+2]);
			}
			js_string+="] );";


			js_string+="var quaternionKF = new THREE.QuaternionKeyframeTrack( '.quaternion', [";
			//timestamp reading
			for(int i=0;i<plant_animation_params.size();i++)
			{
				if(i!=0)
				{
					js_string+=",";
				}
				auto& time_stamp_params = plant_animation_params[i];
				js_string+=std::to_string(time_stamp_params.back());
			}
			js_string+="],[";

			for(int i=0;i<plant_animation_params.size();i++)
			{
				if(i!=0)
				{
					js_string+=",";
				}
				auto& time_stamp_params = plant_animation_params[i];
				js_string+=std::to_string(time_stamp_params[body_index*7+3]);
				js_string+=",";
				js_string+=std::to_string(time_stamp_params[body_index*7+4]);
				js_string+=",";
				js_string+=std::to_string(time_stamp_params[body_index*7+5]);
				js_string+=",";
				js_string+=std::to_string(time_stamp_params[body_index*7+6]);
			}
			js_string+="] );";

			js_string+="var clip = new THREE.AnimationClip( 'Action', -1, [positionKF,quaternionKF] );";
			js_string+="var mixer = new THREE.AnimationMixer( mesh );";
			js_string+="var clipAction = mixer.clipAction( clip );";
			js_string+="clipAction.play();";
			js_string+="mixers.push(mixer);\n";

			body_index++;
			fout<<js_string;
		}

		fout<<html_footer;
		fout.close();
	}
}
