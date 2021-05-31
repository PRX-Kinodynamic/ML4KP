#include "prx/utilities/heuristics/medial_axis.hpp"

#include <fstream>
#include <string>
#ifdef __cpp_lib_filesystem
	#include <filesystem>
	using std::filesystem::exists;
#elif __cpp_lib_experimental_filesystem
	#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM
	#include <experimental/filesystem>
	using std::experimental::filesystem::exists;
#endif

// using namespace std::complex_literals;
namespace prx
{
	constexpr std::complex<double> ma_cell::z_obstacle;// = std::complex<double>(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
	
	void medial_axis_t::set_map(int x_grid, int y_grid, valid_state_t& valid_state, mapping_f f,
		space_t* state_space, std::string _map_name)
	{
		prx_assert(_map_name != "", "Not a valid name for the map!");
		prx_assert(state_space != nullptr, "State space is null!");

		map_file_name = _map_name;
		
		std::ofstream ofs_map;
		ofs_map.open(map_file_name.c_str(), std::ofstream::trunc);

		space_point_t space_point = state_space -> make_point();
		bool vs;
		
		for (int i = 0; i < state_space -> get_dimension(); ++i)
        {
            space_point -> at(i) = 0.0;
        }

        ofs_map << "type octile" << std::endl;
		ofs_map << "height " << x_grid << std::endl;
		ofs_map << "width " << y_grid << std::endl;
		ofs_map << "map" << std::endl;
		for (int i = 0; i < x_grid; i++)
		{
			// std::vector<ma_cell> row;
			ma_map.push_back(std::vector<ma_cell>());
			for (int j = 0; j < y_grid; j++)
			{
				f(i,j,space_point);
				vs = valid_state(space_point);
				ofs_map << (vs?".":"@");
				ma_map.back().push_back(ma_cell());
    	    	set_obstacle(i,j, !vs);
			}
			ofs_map << std::endl;
		}
		height = x_grid;
		width  = y_grid;
		ofs_map.close();
    	stage = map_set;

	}
	
	void medial_axis_t::set_map(const std::string map_file)
	{
		map_file_name = map_file;
    	std::ifstream ifs(map_file_name);
    	std::string line;
    
    	std::getline(ifs,line); // Assuming map_close type is octile, so skip
    
    	std::getline(ifs,line);
    	int i = 0;
    	for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
    	line = line.substr(i,line.length() - i);
    	height = std::atoi(line.c_str());

    	std::getline(ifs,line);    
    	for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
    	line = line.substr(i,line.length() - i);
    	width = std::atoi(line.c_str());

    	std::getline(ifs,line); // This is the line that says "map_close"

    	init_ma_map();
    	for (i = 0; i < height; i++)
    	{
    	    std::getline(ifs,line);
    	    for (int j = 0; j < width; j++)
    	    {
    	    	set_obstacle(i,j, line[j] != '.');
    	    }
    	}
    	stage = map_set;
	}

	void medial_axis_t::add_obstacles_to_graph()
	{
		std::vector<double> pt;

		std::function<bool(int, int)> surrounded_by_obstacles = [&](int i, int j)
    	{
    		// Vec contains the 8 points arround (i,j):
    		// (i--, j--) (i--, j ) (i--, j++)
    		// ( i , j--) ( i , j ) ( i , j++)
    		// (i++, j--) (i++, j ) (i++, j++)
    		// return true iff ALL points are obstacles (or out of bound)
    		auto vec = {std::make_pair( i-1, j-1 ), std::make_pair( i-1, j ), std::make_pair( i-1, j+1),
    					std::make_pair(  i , j-1 ), 						  std::make_pair(  i , j+1),
    					std::make_pair( i+1, j-1 ), std::make_pair( i+1, j ), std::make_pair( i+1, j+1)};
    		for (auto p : vec)
    		{
    			if (p.first < 0 || p.second < 0 || p.first >= rows() || p.second >= cols() ) continue;
    			if (!( is_obstacle(p.first, p.second))) return false;
    		}
    		return true;
    	};

    	// Only add the border of each obstacle
    	for (int i = 0; i <  rows(); ++i)
    	{
    		for (int j = 0; j <  cols(); ++j)
    		{
    			// If this pt is sorrounded by obstacles, no need to add it to the gnn of obstacles
    			if (   is_obstacle(i, j) && !surrounded_by_obstacles(i,j) ) 
    			{
    	    		pt.clear();
    				pt.push_back(i);
    	    		pt.push_back(j);
    	    		auto node_index = obstacles_graph.add_vertex<undirected_node_t,undirected_edge_t>();
					auto node = obstacles_graph.get_vertex_as<undirected_node_t>(node_index);
					node -> point = state_space -> make_point();
					state_space -> copy_point_from_vector(node -> point, pt);
					metric_obstacles -> add_node(node.get());

    			}
    		}
    	}

    	// Add borders as (internal) obstalces 
    	for (int i = -1; i <  rows() + 1; ++i)
    	{
    	    pt.clear();
    		pt.push_back(i);
			pt.push_back(-1);
			auto node_index = obstacles_graph.add_vertex<undirected_node_t,undirected_edge_t>();
			auto node = obstacles_graph.get_vertex_as<undirected_node_t>(node_index);
			node -> point = state_space -> make_point();
			state_space -> copy_point_from_vector(node -> point, pt);
			metric_obstacles -> add_node(node.get());

    	    pt.clear();
    	    pt.push_back(i);
			pt.push_back( cols());
			node_index = obstacles_graph.add_vertex<undirected_node_t,undirected_edge_t>();
			node = obstacles_graph.get_vertex_as<undirected_node_t>(node_index);
			node -> point = state_space -> make_point();
			state_space -> copy_point_from_vector(node -> point, pt);
			metric_obstacles -> add_node(node.get());
    	}
    	for (int i = -1; i <  cols() +1; ++i)
    	{
    	    pt.clear();
			pt.push_back(-1);
    		pt.push_back(i);
			auto node_index = obstacles_graph.add_vertex<undirected_node_t,undirected_edge_t>();
			auto node = obstacles_graph.get_vertex_as<undirected_node_t>(node_index);
			node -> point = state_space -> make_point();
			state_space -> copy_point_from_vector(node -> point, pt);
			metric_obstacles -> add_node(node.get());
    	    pt.clear();
			pt.push_back( rows());
    	    pt.push_back(i);
			node_index = obstacles_graph.add_vertex<undirected_node_t,undirected_edge_t>();
			node = obstacles_graph.get_vertex_as<undirected_node_t>(node_index);
			node -> point = state_space -> make_point();
			state_space -> copy_point_from_vector(node -> point, pt);
			metric_obstacles -> add_node(node.get());
    	}

	}

	void medial_axis_t::set_sknw(const std::string nodes_file, const std::string edges_file, const std::string sknw_file)
	{
		std::ifstream ifs_n(nodes_file);
		std::ifstream ifs_s(sknw_file);
		std::ifstream ifs_e(edges_file);

		std::unordered_map<unsigned long, node_index_t> nodes;
		std::unordered_map<unsigned long, std::vector<std::complex<double>>> edges;
		std::vector<double> pt;

		for (std::string line; std::getline(ifs_e, line); ) 
    	{
    		std::stringstream ss(line);
    		std::istream_iterator<std::string> begin(ss);
			std::istream_iterator<std::string> end;
			std::vector<std::string> vstrings(begin, end);

			prx_assert(vstrings.size() >= 3, "Not enough indexes on a line of file: " << edges_file );

			unsigned long e_index = std::stoul(vstrings[0]);
			edges[e_index].push_back(std::complex<double>(std::stod(vstrings[1]), std::stod(vstrings[2])));

    	}
		// Get nodes from a file of the form:
		// INDEX X1 X2 (...) Xn
    	for (std::string line; std::getline(ifs_n, line); ) 
    	{
    		std::stringstream ss(line);
			std::istream_iterator<std::string> begin(ss);
			std::istream_iterator<std::string> end;
			std::vector<std::string> vstrings(begin, end);
			unsigned long index;
			prx_assert(vstrings.size() >= 2, "Not enough indexes on a line of file: " << nodes_file );

			index = std::stoul(vstrings[0]);
			for (int i = 1; i < vstrings.size(); ++i)
			{
				pt.push_back(std::stod(vstrings[i]));
			}
			auto node_index = graph.add_vertex<undirected_node_t,undirected_edge_t>();
			auto node = graph.get_vertex_as<undirected_node_t>(node_index);
			node -> point = state_space -> make_point();
			state_space -> copy_point_from_vector(node -> point, pt);
			metric_close -> add_node(node.get());
			nodes[index] = node_index;
			set_as_node(std::floor(pt.at(0)), std::floor(pt.at(1)), node_index);
			pt.clear();
    	}

    	for (std::string line; std::getline(ifs_s, line); ) 
    	{
    		std::stringstream ss(line);
			std::istream_iterator<std::string> begin(ss);
			std::istream_iterator<std::string> end;
			std::vector<std::string> vstrings(begin, end);
			prx_assert(vstrings.size() >= 2, "Not enough indexes on a line of file: " << sknw_file);

			node_index_t index_1 = nodes[std::stoul(vstrings[0])];
			node_index_t index_2 = nodes[std::stoul(vstrings[1])];
			auto node_1 = graph.get_vertex_as<undirected_node_t>(index_1);
			auto node_2 = graph.get_vertex_as<undirected_node_t>(index_2);

			auto edge_pts = edges[std::stoul(vstrings[2])];

			std::function<void(node_index_t, node_index_t, int, int)> add_new_node = [&](node_index_t n1, node_index_t n2, int edge1_index, int edge2_index)
			{				

				node_1 = graph.get_vertex_as<undirected_node_t>(n1);
				node_2 = graph.get_vertex_as<undirected_node_t>(n2);
				if ( edge1_index + 1 == edge2_index  || edge1_index == edge2_index 
					|| has_direct_line_of_sight(node_1 -> point, node_2 -> point, los_clearance) )
				{
					auto edge_index = graph.add_edge(n1, n2);
					auto new_edge = graph.get_edge_as<undirected_edge_t>(edge_index);
					new_edge -> set_value( df_ma(node_1 -> point, node_2 -> point) );
					for (int i = edge1_index; i < edge2_index; ++i)
					{
						nodes_edges[edge_index] = std::make_pair(node_1 -> get_index(), node_2 -> get_index());
						set_as_edge(edge_pts[i].real(), edge_pts[i].imag(), edge_index);
						set_as_edge(edge_pts[i].real(), edge_pts[i].imag()+1, edge_index);
						set_as_edge(edge_pts[i].real(), edge_pts[i].imag()-1, edge_index);
						set_as_edge(edge_pts[i].real()+1, edge_pts[i].imag(), edge_index);
						set_as_edge(edge_pts[i].real()-1, edge_pts[i].imag(), edge_index);

					}
					edges_list[node_1 -> get_index()].push_back(edge_index);
					edges_list[node_2 -> get_index()].push_back(edge_index);
					
				}
				else
				{
					// printf("Creating new node\n");
					int middle = edge1_index + (edge2_index - edge1_index)/2;
					// prx_assert( (candidate/2) != (candidate + (edge_pts.size() - candidate) / 2), "Can't connect nodes!");
					auto node_index = graph.add_vertex<undirected_node_t,undirected_edge_t>();
					auto node = graph.get_vertex_as<undirected_node_t>(node_index);
					node -> point = state_space -> make_point(); 

    				std::vector<double> v = {edge_pts[middle].real(), edge_pts[middle].imag()};
    				// unset_edge(edge_pts[middle].real(), edge_pts[middle].imag());
    				set_as_node(edge_pts[middle].real(), edge_pts[middle].imag(), node_index);
    				next_node_id++;
    				// std::cout << v[0] << " " << v[1] << std::endl;
					state_space -> copy_point_from_vector(node -> point, v);
					metric_close -> add_node(node.get());

					add_new_node(n1, node_index, edge1_index, middle);
					add_new_node(node_index, n2, middle, edge2_index);
				}
			};

				add_new_node(index_1, index_2, 0, edge_pts.size());

    	}
    	// stage = sknw_set;
	} 

	// TODO: Remove this function (check that something else is not using it...)
	void medial_axis_t::compute_sknw_and_save_to_files(std::string nodes_file, std::string edges_file, std::string sknw_file, std::string py_sknw, std::string py_cmd)
	{
		prx_assert(goal != nullptr, "Goal not set!");

        std::string cmd = py_cmd + " " + py_sknw + " " + map_file_name + " " + nodes_file + " " + edges_file + " " + sknw_file;
        int res = std::system(cmd.c_str());
        prx_assert(res == 0, "Python command failed");
    	
        set_sknw(nodes_file, edges_file, sknw_file);
    	stage = sknw_set;

	}

	void medial_axis_t::add_goal_to_graph()
	{
		auto create_edges = [&](space_point_t p1, space_point_t p2, node_index_t other, edge_index_t edge_i)
		{
			std::complex<double> c1(std::floor(p1 -> at(0)), std::floor(p1 -> at(1)));
			std::complex<double> c2(std::floor(p2 -> at(0)), std::floor(p2 -> at(1)));
			std::complex<double> c12 = (c2 - c1)/std::abs(c2 - c1);

			 set_as_edge(std::floor(c1.real()), std::floor(c1.imag()), edge_i);
			 	nodes_edges[edge_i] = std::make_pair(goal_index, other);
			while (std::abs(c1-c2) > 1.0)
			{

				c1 += c12;
			 	set_as_edge(std::floor(c1.real()), std::floor(c1.imag()), edge_i);
			 	nodes_edges[edge_i] = std::make_pair(goal_index, other);
			}


		};
		auto vec = metric_close -> multi_query(goal, 20);

		goal_index = graph.add_vertex<undirected_node_t,undirected_edge_t>();
		auto goal_node = graph.get_vertex_as<undirected_node_t>(goal_index);
		goal_node -> point = goal;
		metric_close -> add_node(goal_node.get());
		set_as_node(std::floor(goal -> at(0)), std::floor(goal -> at(1)), goal_index);
		// set_as_node(pt, goal_index);

		for (auto n : vec)
		{
			auto node = static_cast<undirected_node_t*>(n);

			if ( is_obstacle(node -> point -> at(0), node -> point -> at(1))) continue;
			if ( node -> point -> at(0) == goal -> at(0) && node -> point -> at(1) == goal -> at(1) ) continue;
			if (has_direct_line_of_sight(goal, node -> point, los_clearance))
			{
				auto edge_index = graph.add_edge(goal_index, node -> get_index());
				auto new_edge = graph.get_edge_as<undirected_edge_t>(edge_index);
				new_edge -> set_value( df_ma(goal, node -> point) );
				create_edges(node->point, goal, node -> get_index(), edge_index);
				set_as_node(std::floor(node -> point -> at(0)), std::floor(node -> point -> at(1)), node -> get_index());
				edges_list[node -> get_index()].push_back(edge_index);
				edges_list[goal_index].push_back(edge_index);

			}
		}
	}

	void medial_axis_t::prepare_graph()
	{
		prx_assert(stage >= sknw_set, "To prepare graph, sknw has to be set/computed.");
    	compute_distances_to_ma();

		add_goal_to_graph();

    	graph.dijkstra(goal_index);
		graph.vertex_list_to_file(lib_path + "/out/ma_graph_dijkstra.txt");

    	stage = graph_ready;
	}

	void medial_axis_t::compute_vector_fields()
	{
		prx_assert(false, "Function not usable");
		space_point_t pt;
		pt =  state_space -> make_point();
		std::complex<double> best_far;
		std::complex<double> best_close;
		double best_cost, min_dist, aux_dist;
		undirected_node_t* node;
		for (int i = 0; i <  rows(); ++i)
		{

			for (int j = 0; j <  cols(); ++j)
			{
			}
		}
		stage = far_ready;
	}

	void medial_axis_t::find_medial_axis()
	{
		std::complex<double> vp;
		std::complex<double> v;

		std::vector<std::complex<double>> possible_edges;
		
		// std::function<bool(ma_pt, ma_pt)> ma_pt_cmp = [](ma_pt a, ma_pt b)
		// {return (std::hash<double>{}(a.real()) ^ (std::hash<double>{}(a.imag()) << 1) ) < 
		// 	(std::hash<double>{}(b.real()) ^ (std::hash<double>{}(b.imag()) << 1) );};

		// std::set<ma_pt,decltype(ma_pt_cmp)> nodes(ma_pt_cmp);
		std::vector<ma_pt> nodes;


		ma_pt uv;

		std::function<double(ma_pt, ma_pt)> dot = [](ma_pt a, ma_pt b)
		{
			return a.real() * b.real() + a.imag() * b.imag();
		};

		// Check if the current coordinate should be part of the Medial Axis
		// by checking if there is a change on direction of the surrounding vectors
		std::function<bool(int, int)> is_ma = [&](int i, int j)
		{
			std::vector<ma_pt> vs;
			ma_pt c = -map_close(i,j);
			// if (0 <= i-1 && 0 <= j-1 && i-1 < rows() && j-1 < cols() && !is_obstacle(i-1, j-1)) vs.push_back(-map_close(i-1,j-1));
			if (0 <= i-1 && 0 <= j+0 && i-1 < rows() && j+0 < cols() && !is_obstacle(i-1, j+0)) vs.push_back(-map_close(i-1,j+0));
			// if (0 <= i-1 && 0 <= j+1 && i-1 < rows() && j+1 < cols() && !is_obstacle(i-1, j+1)) vs.push_back(-map_close(i-1,j+1));
			if (0 <= i+0 && 0 <= j-1 && i+0 < rows() && j-1 < cols() && !is_obstacle(i+0, j-1)) vs.push_back(-map_close(i+0,j-1));
			// if (0 <= i+0 && 0 <= j+0 && i+0 < rows() && j+0 < cols() && !is_obstacle(i+0, j+0)) vs.push_back(-map_close(i+0,j+0));
			if (0 <= i+0 && 0 <= j+1 && i+0 < rows() && j+1 < cols() && !is_obstacle(i+0, j+1)) vs.push_back(-map_close(i+0,j+1));
			// if (0 <= i+1 && 0 <= j-1 && i+1 < rows() && j-1 < cols() && !is_obstacle(i+1, j-1)) vs.push_back(-map_close(i+1,j-1));
			if (0 <= i+1 && 0 <= j+0 && i+1 < rows() && j+0 < cols() && !is_obstacle(i+1, j+0)) vs.push_back(-map_close(i+1,j+0));
			// if (0 <= i+1 && 0 <= j+1 && i+1 < rows() && j+1 < cols() && !is_obstacle(i+1, j+1)) vs.push_back(-map_close(i+1,j+1));

			if (vs.size() <= 1) return true;

			double cn = std::abs(c);
			double ang = 0;
			for (int k = 0; k < vs.size(); ++k)
			{

				// MA_DEBUG(i, j, 181, 350, "c: " << c << " vs[k]: " << vs[k] << " angle: " << ang << " count: " << ma_map[i][j].close_count);
				if (c == -vs[k] ) return true; // std::abs(std::arg(vs[i] + vs[j]));
				ang = std::abs(std::acos(dot(c, vs[k]) / (cn * std::abs(vs[k]))));
				if (ang >= M_PI / 2.0 ) return true; // std::abs(std::arg(vs[i] + vs[j]));
				// if (ma_map[i][j].close_count > 1 && ang >= M_PI / 4.0) return true;
				// if (ma_map[i][j].close_count > 1 && ang == M_PI / 2.0) return true;
			
				// for (int l = k+1; l < vs.size(); ++l)
				// {
				// 	if (std::abs(vs[k]) == std::abs(vs[l])) return true;

				// // 	if (vs[k] == -vs[l] ) return true; // std::abs(std::arg(vs[i] + vs[j]));
				// // 	else if (std::abs(std::acos(dot(vs[k], vs[l]) / (std::abs(vs[k]) * std::abs(vs[l])))) >= M_PI / 2.0 ) return true; // std::abs(std::arg(vs[i] + vs[j]));
				// }
			}
			// MA_DEBUG(i, j, 181, 350, "returning false");

			// return cont >= vs.size();
			// return true;
			return false;
		};

		std::function<double(double, double)> angle_diff = [&](double a1, double a2)
		{
			return std::fabs(a1 - a2);
			// return std::min(std::fabs(a1 - a2), std::fabs(norm_angle_pi(a1+M_PI, -M_PI, M_PI) - a2 ));
		};

	 	ma_pt init;
	 	int i, j;
		std::function<void(ma_pt, double)> set_MA = [&](ma_pt pt, double dist_prev)
		{
			// std::cout << "init: " << init << std::endl;
			// if (is_edge(EXPAND_CMP(pt))) return false;
			// if (is_obstacle(EXPAND_CMP(pt))) return false;
			// if (!in_bounds(EXPAND_CMP(pt))) return false;
			// double dist_curr = dist_to_obstacle(pt);
			// // double sin_angle = std::sin(std::arg(pt));
			// double curr_angle = std::arg(pt - init);
			// if (dist_curr <= dist_prev) return true;
			// // MA_DEBUG(i,j, 10, 10, "curr_angle: " << curr_angle << " dist_prev: " << dist_prev << " distance: " << angle_diff(curr_angle, dist_prev));
			// // MA_DEBUG(pt.real(), pt.imag(), 10, 10, "sin_angle: " << sin_angle << " dist_prev: " << dist_prev);
			// set_as_edge(EXPAND_CMP(pt));
			
			// auto close = close_vector_at(EXPAND_CMP(pt));
			// auto close_ang = std::arg(close);
			// std::cout << "\tpt: " << pt << " curr_angle: " << curr_angle << " close_ang: " << close_ang << " diff: " << angle_diff(curr_angle, close_ang) << std::endl;
			// auto cv = pt + close ;// close_vector_at(EXPAND_CMP(pt));
			// std::cout <<"\tclose: " << close_vector_at(EXPAND_CMP(pt)) << " cv: " << cv << std::endl;

			// if ( set_MA(cv, dist_curr)) 
			// {
			// std::cout << "\t\tTRUE!" << std::endl;

			// 	nodes.push_back(cv);

			// }
			// return false;
			bool first = true;
			ma_pt first_obst = pt -close_vector_at(EXPAND_CMP(pt), false);
			for (auto d : dir4)
			{
				auto adj = pt + d.first;
				// std::cout << "pt: " << pt << " adj: " << adj << " obst: " << first_obst << std::endl;
				if (is_obstacle(EXPAND_CMP(adj))) continue;
				if (!in_bounds(EXPAND_CMP(adj))) continue;
				auto close = -close_vector_at(EXPAND_CMP(adj), false);
				auto v = adj + close;
				// std::cout << "\tclose: " << close << " v: " << v << " diff: " << std::fabs(std::abs(first_obst) - std::abs(v)) << std::endl;
				// if (first)
				// {
				// 	first = false;
				// 	first_obst = v;
				// }
				if ( first_obst == v ) continue;
				if ( std::fabs(std::abs(first_obst - v)) >= 1.5  )
				{
					// std::cout << "\t\tFOUND!" << std::endl;
					nodes.push_back(pt);
					return;
				}

			}
		};

		// const std::string file_to_ma = lib_path + "/out/ma_map_" + "simple_obstacle" + "_vf_to_ma.txt";
		// std::ofstream ofs_map;
		// ofs_map.open(file_to_ma.c_str(), std::ofstream::trunc);
		for (i = 0; i < rows(); ++i)
		{
			for (j = 0; j < cols(); ++j)
			{
				if (is_obstacle(i, j) ) continue;
				// if (is_ma(i,j)) nodes.push_back(std::complex<double>(i,j));
				// MA_DEBUG(i, j, 460, 233, "is_node: " << is_node(i,j));
				// MA_DEBUG(i, j, 461, 233, "is_node: " << is_node(i,j));
				// if (ma_map[i][j].close_count > 1)
				// if (is_node(i,j))
				// {
			// MA_DEBUG(i,j, 10, 10, "Debuging...");

				// 	nodes.push_back(std::complex<double>(i,j));
				// 	set_node_as(i, j, false);
				// } 
				// set_MA()
					set_node_as(i, j, false);

				init = std::complex<double>(i,j);
				auto pt = init + close_vector_at(EXPAND_CMP(init));
			
			set_MA(init, 0);
			// if ( set_MA(pt, std::arg(pt)) ) 
			// {
			// 	nodes.push_back(pt);

			// }

			}
		}
		// for (int i = 0; i < obstacles_graph.num_vertices(); ++i)
		// {
		// 	auto pt = obstacles_graph[i] -> point;
		// 	std::complex<double> p(pt -> at(0), pt -> at(1));
		// 	for (auto d : dirs)
		// 	{
		// 		set_MA(p + d.first, 0);
		// 	}
		// }
		// PRX_DEBUG_PRINT
		// std::set<ma_pt,decltype(ma_pt_cmp)> visited(ma_pt_cmp);
		std::set<node_index_t> nodes_visited;

		auto add_node_to_graph = [&](ma_pt pt)
		{
			if (is_node(pt)) return get_node_id(pt);

			auto node_index = graph.add_vertex<undirected_node_t,undirected_edge_t>();
			auto node = graph.get_vertex_as<undirected_node_t>(node_index);
			node -> point = state_space -> make_point();
			state_space -> copy_point_from_vector(node -> point, {pt.real(), pt.imag()});

			metric_close -> add_node(node.get());
			nodes_visited.insert(node_index);
			set_as_node(pt, node_index);

			return node_index;
		};

		std::complex<double> r;
		node_index_t n1, n2;

		for (auto e : nodes)
		{
			n1 = add_node_to_graph(e);

			// n1 = get_node_id(e);

			for (auto d : dir4)
			{
				r = e + d.first;

				if (is_node(r))
				{
					n2 = get_node_id(r);
					auto node = graph.get_vertex_as<undirected_node_t>(n1);
					if ( ! node -> is_neighbor(n2) )
					{
						auto edge_index = graph.add_edge(n1, n2, d.second);
					}
				}	
			}

		}
		
		// std::cout << "total metric numbers: " << metric_close -> get_nr_nodes() << std::endl;

    	stage = sknw_set;

	}

	void medial_axis_t::compute_close_vector(int i, int j)
	{
		// Skip where there is an obstacle
		if ( is_obstacle(i,j) ) return;
				
		// pt <- (i,j)
		pt -> at(0) = i;
		pt -> at(1) = j;

		double aux_dist;
		undirected_node_t* node;
		// std::set<undirected_node_t*> vec_nodes;
		double min_dist = std::numeric_limits<double>::infinity();
		std::vector<proximity_node_t*> vec_nodes = metric_close -> multi_query(pt, 30);// radius_and_closest_query(pt, std::abs(map_close(i,j)));

		for (int i = 0; i < 5; ++i)
		{
			node = static_cast<undirected_node_t*>(metric_obstacles -> single_query(pt));
			aux_dist = df_ma(node -> point, pt);

			if ( aux_dist <= min_dist )
			{
				min_dist = aux_dist;
				vec_nodes.push_back(node);
				// vec_nodes.insert(node);
			} 
		}
		double dist_aux = 0;

		v1 = std::complex<double>(i, j);	
		v3 = std::complex<double>(0, 0);	
		double cont = 0;

		for (auto n : vec_nodes)
		{
			node =  static_cast<undirected_node_t*>(n);
			dist_aux = df_ma(node -> point, pt);
			if ( dist_aux == min_dist)
			{
				v2 = std::complex<double>(node -> point -> at(0), node -> point -> at(1));
			MA_DEBUG(i, j, 181, 350, "v1: " << v1 << " v2: " << v2 << " v3: " << v3);
				if ( v3 != -(v1 - v2) )
				{
					v3 = (v1 - v2);
					cont++;
					ma_map[i][j].close_count++;

				}
			}
		}

		// v3 = cont == 0 ? v3 : v3 / cont; 
		// MA_DEBUG(0, 0, i, j, "v3: " << v3)
		map_close(i,j) = v3; //std::complex<double>(v3(0), v3(1));
	}

	void medial_axis_t::compute_far_vector(int i, int j)
	{
		prx_assert(stage >= graph_ready, "Close VF has not been computed!");

		if (  is_obstacle(i,j) || !in_bounds(i,j)) return;
		
		// pt <- (i,j)
		pt -> at(0) = i;
		pt -> at(1) = j;

		v1 = std::complex<double>(i, j);	
		v3 = std::complex<double>(0, 0);

		// Test direct line of sight to the goal
		if (has_direct_line_of_sight(pt, goal, los_clearance))
		{
			// v2(0) = goal -> at(0);
			// v2(1) = goal -> at(1);
			v2 = std::complex<double>(goal -> at(0), goal -> at(1));	

		}
		else // Find the medial axis node with direct line of sight from current pos that is closest to the goal 
		{ 
			bool vector_computed = false;
			double best_cost = std::numeric_limits<double>::infinity();
			std::shared_ptr<undirected_node_t> node;
			// undirected_node_t* node;
			
			// Assume no sln
			v2 = std::complex<double>(i, j);	

			std::set<node_index_t> vec_nodes;

			std::vector<proximity_node_t*> vec_close = metric_close -> radius_and_closest_query(pt, dist_to_obstacle(v2));

			if (is_node(i, j)) 
			{
				auto pn = graph.get_vertex_as<undirected_node_t>(get_node_id(i, j));
				vec_nodes.insert(pn -> get_best_neighbor());
			}

			std::function<void(undirected_node_t*)> insert_nodes = [&](undirected_node_t* ni)
			{	
				if (vec_nodes.count(ni -> get_index()) > 0) return;
				if (has_direct_line_of_sight(pt, ni -> point, los_clearance))
				{
					vec_nodes.insert(ni -> get_index());
					vec_close.push_back(static_cast<proximity_node_t*>(ni));
					auto next_node = graph.get_vertex_as<undirected_node_t>(ni -> get_best_neighbor());
					// insert_nodes(static_cast<undirected_node_t*>( ) );
					insert_nodes(next_node.get());
				}
			};

			while (vec_close.size() > 0)
			{
				insert_nodes(static_cast<undirected_node_t*>(vec_close[0]));
				vec_close.erase(vec_close.begin());

			}
			for (auto n : vec_nodes)
			{
				node = graph.get_vertex_as<undirected_node_t>(n);
				// MA_DEBUG(i, j, 389, 77, "n: " << node -> point -> at(0) << " " << node -> point -> at(1) <<
				//  " cost: " << node -> get_cost_to_go() << " best_cost: " << best_cost << " dist: " << df_ma(pt, node -> point));
				if (n == get_node_id(i, j)) continue;
				if ( df_ma(pt, node -> point) + node -> get_cost_to_go() < best_cost)// &&
				{
					best_cost = df_ma(pt, node -> point) + node -> get_cost_to_go();

					v2 = std::complex<double>(node -> point -> at(0), node -> point -> at(1));	

					vector_computed = true;
				}

			}
			// If no vector with the specified clearance was found, recompute with 
			// clearance of 0.
			if ( best_cost == std::numeric_limits<double>::infinity() )
			{

				// if (i == 188 && j == 58) printf("clearance = 1\n" );
				// for (auto n : vec_nodes)
				for (auto n : vec_nodes)
				{
				node = graph.get_vertex_as<undirected_node_t>(n);
					// node = static_cast<undirected_node_t*>(n);
					if ( df_ma(pt, node -> point) + node -> get_cost_to_go() < best_cost &&
							has_direct_line_of_sight(pt, node -> point, 0.0))
					{
						best_cost = df_ma(pt, node -> point) + node -> get_cost_to_go();
						// v2(0) = node -> point -> at(0); 
						// v2(1) = node -> point -> at(1);
						v2 = std::complex<double>(node -> point -> at(0), node -> point -> at(1));	
						vector_computed = true;

					}
				}
			}
			// If a sln has not been found yet, just point to the closer without checking 
			// line of sight
			if ( best_cost == std::numeric_limits<double>::infinity() )
			{
				auto node_ptr = static_cast<undirected_node_t*>(metric_close -> single_query(pt));	
				// v2(0) = node -> point -> at(0); 
				// v2(1) = node -> point -> at(1);
				v2 = std::complex<double>(node_ptr -> point -> at(0), node_ptr -> point -> at(1));	

			}
		}
		// MA_DEBUG(i, j,389, 77, "v2: " << v2);

		v3 = (v2 - v1);

		map_far(i,j) = v3;//std::complex<double>(v3(0), v3(1));

	}

	void medial_axis_t::compute_close_vector_field()
	{
		prx_assert(stage >= map_set, "To compute close VF, graph has to be ready (call prepare_graph).");
		add_obstacles_to_graph();

	
		std::deque<std::pair<ma_pt,ma_pt>> cells;
		ma_pt a;
		double dist_aux = 0.0;
		std::function<void(ma_pt, ma_pt)> add_to_cells = [&](ma_pt pt, ma_pt obstacle)
		{
			if ( in_bounds(EXPAND_CMP(pt)) &&
				 !is_obstacle(EXPAND_CMP(pt)) )
			{
				dist_aux = dist_to_obstacle(pt);
				ma_map[pt.real()][pt.imag()].close_count++;
			}
			else
			{
				dist_aux = 0.0;
			}
			for (auto d : dir4)
			{
				a = d.first + pt;
				// auto b = in_bounds(EXPAND_CMP(a));
				// auto c =  !is_obstacle(a.real(), a.imag());
    // std::cout << b << " " << c << " a: " << a << " pt: " << pt << std::endl;
				// auto e = b && c ? dist_to_obstacle(a) : 0.0;
    // PRX_DEBUG_PRINT;
				// auto f = b && c ? dist_to_obstacle(pt) + d.second : 0.0;
    // std::cout << b << " " << c << " " << e << " " << f << std::endl;
				if ( in_bounds(EXPAND_CMP(a)) &&
					 !is_obstacle(a.real(), a.imag()) && 
					 dist_to_obstacle(a) > ( dist_aux + d.second )
					)
				{
					// set_node_as(a.real(), a.imag(), true);
					// if (dist_aux != 0.0)
					// 	set_node_as(pt.real(), pt.imag(), false);
			// std::cout << "pt: " << pt << " pt_dist: " << dist_to_obstacle(pt) << "cell: " << a << " dist: " << dist_to_obstacle(a) << std::endl;
					// ma_map[a.real()][a.imag()].close_count++;

					cells.push_back(std::make_pair(std::complex<double>(EXPAND_CMP(a)), obstacle));
					// map_close(EXPAND_CMP(a)) = a - obstacle;
					if (is_obstacle(EXPAND_CMP(pt)))
					{
						map_close(EXPAND_CMP(a)) = d.first; 
						// set_node_as(a.real(), a.imag(), true);

					}
					else
					{
						map_close(EXPAND_CMP(a)) = map_close(EXPAND_CMP(pt)) + d.first; 
						// set_node_as(a.real(), a.imag(), true);
						// set_node_as(pt.real(), pt.imag(), false);
						// MA_DEBUG(a.real(), a.imag(), 460, 233, "a: " << a << " pt: " << pt << map_close(a.real(), a.imag()));
						// MA_DEBUG(pt.real(), pt.imag(), 460, 233, "a: " << a << " pt: " << pt << map_close(pt.real(), pt.imag()));
						// MA_DEBUG(a.real(), a.imag(), 461, 233, "a: " << a << " pt: " << pt << map_close(a.real(), a.imag()));
						// MA_DEBUG(pt.real(), pt.imag(), 461, 233, "a: " << a << " pt: " << pt << map_close(pt.real(), pt.imag()));
					}
					dist_to_obstacle(a) = dist_aux + d.second;
				} 
			}
		};


		for (int i = 0; i < obstacles_graph.num_vertices(); ++i)
		{
			auto pt = obstacles_graph[i] -> point;
			std::complex<double> p(pt -> at(0), pt -> at(1));
			// std::cout << "obstacle: " << p << std::endl;
			// cells.push_back(std::complex<double>(pt -> at(0), pt -> at(1)));
			if (in_bounds(EXPAND_CMP(p))) 
			{
				dist_to_obstacle(p) = 0;
			}

				add_to_cells(p, p);
		}
		// std::cout << "size: " << cells.size() << std::endl;
		while (cells.size() > 0)
		{
			add_to_cells(cells.front().first, cells.front().second);
			cells.pop_front();

		}
		
		stage = close_ready;
	}

	void medial_axis_t::compute_far_vector_field()
	{
		prx_assert(stage >= graph_ready , "Close VF has not been computed!");
		
		for (int i = 0; i <  rows(); ++i)
		{
			for (int j = 0; j <  cols(); ++j)
			{
				// Skip where there is an obstacle
				compute_far_vector(i,j);
			}
		}
		stage = far_ready;
	}

	void medial_axis_t::close_vf_to_file(std::string file_name, bool normalized )
	{
		prx_assert(stage >= close_ready, "Close VF has not been computed!");

		std::ofstream ofs_map;
		std::cout << "Saving close VF to: " << file_name << std::endl;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);
		std::complex<double> uv;

		for (int i = 0; i <  rows(); ++i)
		{
			for (int j = 0; j <  cols(); ++j)
			{
				uv = close_vector_at(i, j, normalized);

				ofs_map << i << " " << j << " ";
				ofs_map << uv.real() << " " << uv.imag() << std::endl;

			}
		}
		ofs_map.close();
	}


	void medial_axis_t::far_vf_to_file(std::string file_name, bool normalized )
	{
		prx_assert(stage == far_ready, "Far VF has not been computed!");
		std::cout << "Saving far VF to: " << file_name << std::endl;

		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		std::complex<double> uv;

		for (int i = 0; i <  rows(); ++i)
		{
			for (int j = 0; j <  cols(); ++j)
			{
				// if (is_node(i,j) || is_obstacle(i,j))
				// {
				uv = far_vector_at(i, j, normalized);
				
				ofs_map << i << " " << j << " ";
				ofs_map << uv.real() << " " << uv.imag() << std::endl;
				// }
			}
		}
		ofs_map.close();
	}

	std::complex<double> medial_axis_t::close_vector_at(int i, int j, bool normalized)
	{
		prx_assert(stage >= close_ready, "Close VF has not been computed!");
		double u,v;
		if ( !in_bounds(i,j) ) return ma_cell::z_obstacle;
		if ( is_obstacle(i,j) || map_close(i, j) == v_zero)
		{
			u = map_close(i,j).real();
			v = map_close(i,j).imag();
		}
		else
		{
			double n = normalized ? std::abs(map_close(i,j)) : 1.0;
			u = map_close(i,j).real() / n;
			v = map_close(i,j).imag() / n;
		}
		return std::complex<double>(u, v);
	}
	
	std::complex<double> medial_axis_t::far_vector_at(int i, int j, bool normalized)
	{
		prx_assert(stage >= close_ready, "Far VF has not been computed!");
		double u,v;
		
		if (stage == close_ready)
		{
			compute_far_vector(i, j);
		}
		if ( !in_bounds(i,j) )
		{
			u = std::numeric_limits<double>::max();
			v = std::numeric_limits<double>::max();
		}
		else if ( is_obstacle(i,j) || map_far(i, j) == v_zero)
		{
			u = map_far(i,j).real();
			v = map_far(i,j).imag();
		}
		else
		{
			double n = normalized ? std::abs(map_far(i,j)) : 1.0;
			u = map_far(i,j).real() / n;
			v = map_far(i,j).imag() / n;
		}
		return std::complex<double>(u, v);
	}

	std::vector<std::complex<double>> medial_axis_t::blossom(int i, int j, int n, double max_magnitud, double min_magnitud, bool avg_if_obstacle)
	{
		std::vector<ma_pt> r;
		std::map<node_index_t, ma_pt> vectors;
		std::map<node_index_t, ma_pt> rejected;
		std::vector<double> vec_ids;
		r.push_back(integrated_vector_at(i,j, max_magnitud, min_magnitud, avg_if_obstacle));
		ma_pt start(i,j);
		ma_pt far = far_vector_at(i,j, false);
		ma_pt next;
		pt -> at(0) = i; 
		pt -> at(1) = j;
		// const double max_magnitud = std::abs(far);

		std::function<void(ma_pt)> get_vectors = [&](ma_pt p)
		{
		// MA_DEBUG(i,j, 160, 160, "\tp: " << p )
			if (vectors.count(get_node_id(p)) > 0) return;
			if (rejected.count(get_node_id(p)) > 0) return;
			// if (is_node(p))
			// {
			if (max_magnitud < std::abs(p - start) )
			{
				rejected[get_node_id(p)] = p;
			}
			else
			{
				vec_ids.push_back(get_node_id(p));
				vectors[get_node_id(p)] = p;
			}
				auto node = graph.get_vertex_as<undirected_node_t>(get_node_id(p));
				for (auto ni : node -> get_neighbors())
				{
					auto neighbor = graph.get_vertex_as<undirected_node_t>(ni);

					ma_pt next(neighbor -> point -> at(0), neighbor -> point -> at(1));

					// if (has_direct_line_of_sight(pt, neighbor -> point, 0))					
						get_vectors(next);
				}

		};
		auto sf = start + far;
		if (is_node(sf))
		{
			

			get_vectors(sf);
    		// std::default_random_engine e(seed);
  //   		std::cout << "seed: " << seed << std::endl;
		// PRX_DEBUG_ITERABLE("pre-shuffled", vec_ids);
		// PRX_DEBUG_ITERABLE("shuffled", vec_ids);
		} 

		if ( avg_if_obstacle && is_obstacle(i, j) ) 
		{
			std::complex<double> r(0, 0);
			std::complex<double> b(0, 0);
			double c = 0;
			for (auto d : dirs)
			{
				auto a = d.first + start;
				if (is_obstacle(EXPAND_CMP(a)) || !in_bounds(EXPAND_CMP(a))) continue;
				sf = a + far_vector_at(EXPAND_CMP(a), false);// + d.first;
				// MA_DEBUG(i,j,170,5, "sf: " << sf << " a: " << a << " is_node: " << is_node(sf));
				if (is_node(sf))
				{
					get_vectors(sf);
				}
			}
			// return c == 0 ? ma_cell::z_obstacle : clamp(r / c);
		}
		// MA_DEBUG(i,j,170,5, "vec_ids: " << vec_ids.size());

		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::shuffle(vec_ids.begin(), vec_ids.end(), std::default_random_engine(seed));

		for (auto v : vec_ids)
		{
		// MA_DEBUG(i,j, 160, 160, "\tv: " << v.second )
			if (r.size() >= n ) break;
			auto other = graph.get_vertex_as<undirected_node_t>(v);

			if ( avg_if_obstacle && is_obstacle(i, j) ) 
			{
				for (auto d : dirs)
				{
					auto a = d.first + start;
					if (is_obstacle(EXPAND_CMP(a)) || !in_bounds(EXPAND_CMP(a))) continue;

					pt -> at(0) = a.real(); 
					pt -> at(1) = a.imag();
					if (has_direct_line_of_sight(pt, other -> point, 0))
					{
						r.push_back(vectors[v] - start);
						break;
					}
				}
			}

			if (has_direct_line_of_sight(pt, other -> point, 0))					
			{
				r.push_back(vectors[v] - start);
			}
		}
		return r;
	}

	std::complex<double> medial_axis_t::integrated_vector_at(int i, int j, double max_magnitud, double min_magnitud, bool avg_if_obstacle)
	{
		prx_assert(min_magnitud < max_magnitud, "Problem with the requested magnitud range of the vector: [ " << min_magnitud << " , " << max_magnitud << " ]" );
		std::function<bool(ma_pt)> near_ma = [&](ma_pt v)
		{
			// int ii = v.real();
			// int jj = v.imag();

			return is_node( v.real(), v.imag()  );
		};

		// std::function<std::complex<double>(ug_node_ptr, std::complex<double>)> update_v = [&](ug_node_ptr n, std::complex<double> v)
		// {
		// 	if (n -> get_cost_to_go() < best_cost)
		// 	{
		// 		v.real(n -> point -> at(0));
		// 		v.imag(n -> point -> at(1));
		// 		best_cost = n -> get_cost_to_go();
		// 	}
		// 	return v;
		// };

		auto try_connect_to_goal = [&](space_point_t pt)
		{
			std::complex<double> v = ma_cell::z_obstacle;
			if (has_direct_line_of_sight(pt, goal, los_clearance))
			{
				v.real(goal -> at(0));
				v.imag(goal -> at(1));
			}

			return v;
		};

		std::function<std::complex<double>(std::complex<double>)> clamp = [&](std::complex<double> uv)
		{
			double m = std::abs(uv);
			std::complex<double> r;
			if ( m == 0)
			{
				r = v_zero;
			}
			else if ( m > max_magnitud)
			{
				r = uv * max_magnitud / m;
			}
			else 
			{
				r = uv;
			}
			return r;
			// return m > max_magnitud ? uv * max_magnitud / m : uv;
		};

		std::function<std::complex<double>(std::complex<double>)> fancy_grow = [&](std::complex<double> v)
		{
			std::complex<double> init(i, j);

			std::complex<double> r = v;
			std::complex<double> point = v_zero;
			int limit = 0;
			// v = u + v;
			while (std::abs(r) < min_magnitud && limit < min_magnitud*2)
			{
				point = init + r;
				if (is_obstacle(EXPAND_CMP(point))) break;
				if (!in_bounds(EXPAND_CMP(point))) break;
				// std::cout << init << " r: " << r;
				// std::cout << " magnitud: " << std::abs(r) << " point: " << point;
				// std::cout << " map_far: " << map_far(EXPAND_CMP(point)) << std::endl;
				r += far_vector_at(EXPAND_CMP(point));
				// r += map_far(EXPAND_CMP(point));
				// p += r;
				limit++;
			}
			return r;
		};

		std::complex<double> u(i, j);

		if ( avg_if_obstacle && is_obstacle(i, j) ) 
		{
			std::complex<double> r(0, 0);
			std::complex<double> b(0, 0);
			double c = 0;
			for (auto d : dirs)
			{
				auto a = d.first + u;
				if ( !in_bounds(EXPAND_CMP(a)) ) continue;

				b = close_vector_at(EXPAND_CMP(a), false) ;
				if (b != ma_cell::z_obstacle)
				{
					r += ( b * get_dist_to_ma(EXPAND_CMP(a)));
					c++;
				}
			}
			return c == 0 ? ma_cell::z_obstacle : clamp(r / c);
		}
		if ( is_obstacle(i, j) ) return ma_cell::z_obstacle;
		if (get_node_id(i,j) == goal_index) return v_zero;
		auto start = compute_integrated_vector(i,j);
		if (start == v_zero) return v_zero;

		std::complex<double> goal_c(goal -> at(0), goal -> at(1));
		double mult = 0.0;
		double curr_magnitud = 0.0;
		double best_cost = std::numeric_limits<double>::infinity();
		// double ma_cost_to_go = 0.0;		
		std::shared_ptr<undirected_node_t> node;
			// double d_far = get_dist_to_ma(i,j);
			// double d_close = std::max(1.,std::abs(map_close(i,j)));
			// double close_weight = d_far/(d_far + d_close);
			// double far_weight = d_close/(d_far + d_close);
			curr_magnitud = std::abs(start);
			double last_good_mult = 1.0;
			while (true)
			{
				// MA_DEBUG(i,j, 15, 60, "u: " << u << " mult: " << mult << " last_good_mult: " << last_good_mult << " best_cost: " << best_cost);
				if ( near_ma(u) )//&& !near_ma(u + start) )
				{
					node = graph.get_vertex_as<undirected_node_t>(get_node_id(EXPAND_CMP(u)));

					if (node != nullptr && mult > 0 && best_cost >= node -> get_cost_to_go() + mult)
					{
						best_cost = std::ceil(mult + node -> get_cost_to_go());
						last_good_mult = mult; // break;
					}
				}
				// if (mult >= max_magnitud ) break;
				if (std::abs(goal_c - u) <= 1) {last_good_mult = mult; break;}
				// MA_DEBUG(i, j, 14, 6, "u: " << u << " was_near_ma: " << near_ma(u) << " near_ma: " << near_ma(u + start));
				// if ( was_near_ma && !near_ma(std::floor(u.real()), std::floor(u.imag())) ) break;
				if (!in_bounds(std::floor(u.real()), std::floor(u.imag())))  break;// return clamp(start * last_good_mult);
				if (is_obstacle(std::floor(u.real()), std::floor(u.imag()))) break;// return clamp(start * last_good_mult);
				// if ( best_cost != std::numeric_limits<double>::infinity() && get_dist_to_ma(EXPAND_CMP(u)) > dist_to_obstacle(u)) break;
				// if ( best_cost != std::numeric_limits<double>::infinity() && std::min(get_dist_to_ma(EXPAND_CMP(u)), los_clearance) > dist_to_obstacle(u) ) break;
				// if ( std::min(mult, los_clearance) > dist_to_obstacle(u) ) break;
				if ( mult > los_clearance && std::min(los_clearance, get_dist_to_ma(EXPAND_CMP(u))) > dist_to_obstacle(u) ) break;
				
				// was_near_ma = near_ma(std::floor(u.real()), std::floor(u.imag()));
				// curr_magnitud += 1.0;// std::abs(start * mult);

				u += start;
				mult += 1.0;
			}
		mult = best_cost == std::numeric_limits<double>::infinity() ? mult : last_good_mult;
		return fancy_grow(clamp(start * mult));
	}

	std::complex<double> medial_axis_t::compute_integrated_vector(int i, int j)
	{
		prx_assert(stage >= close_ready, "Close VF has not been computed!");
		double u, v;
		if (stage < far_ready)
		{
			compute_far_vector(i, j);
		}

		std::complex<double> v_far, v_close;
		v_close = v_zero;
		v_far = v_zero;
		double weight = 1;
		if (  is_obstacle(i,j) )
		{
			return ma_cell::z_obstacle;

		}
		else
		{
			double d_far = get_dist_to_ma(i,j);
			double d_close = std::max(1.,std::abs(map_close(i,j)));
			double clearance = std::max(1.,los_clearance);
			double close_weight = d_far/(d_far + d_close);
			double far_weight = d_close/(d_far + d_close);
			// printf("close_weight: %.4f\tfar_weight: %.4f\n", close_weight, far_weight );
			if ( map_close(i, j) != v_zero )
			{
				v_close = map_close(i,j) * close_weight / (std::abs(map_close(i,j))) ;
			}
			if( map_far(i, j) != v_zero )
			{
				v_far = map_far(i,j) * far_weight / (std::abs(map_far(i,j)));
			}

		}
		
		return v_close + v_far;
	}

	double medial_axis_t::cost_to_go(int i, int j)
	{
		auto far = far_vector_at(i,j, false);
		ma_pt init(i, j);
		if (far == ma_cell::z_obstacle)
		{

			for (auto d : dirs)
			{
				auto a = d.first + init;
				if (is_obstacle(EXPAND_CMP(a)) || !in_bounds(EXPAND_CMP(a))) continue;
				return cost_to_go(EXPAND_CMP((d.first + init))) + d.second;
			}
			return std::numeric_limits<double>::max();
		}
		auto sf = init + far;
		if (!is_node(sf)) return std::numeric_limits<double>::max();

		auto node = graph.get_vertex_as<undirected_node_t>(get_node_id(sf));
		return std::abs(far) + node -> get_cost_to_go();
	}

	void medial_axis_t::cost_to_go_to_file(std::string file_name)
	{
		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		for (int i = 0; i <  rows(); ++i)
		{
			for (int j = 0; j <  cols(); ++j)
			{
				// uv = integrated_vector_at(i,j, 100, 10, true);
				ofs_map << i << " " << j << " ";
				ofs_map << cost_to_go(i,j) << std::endl;
			}
		}
		ofs_map.close();
	}

	void medial_axis_t::set_far_vf_from_file(std::string file_name)
	{
		prx_assert(stage == map_set, "Close VF not set.");
		vector_field_from_file(file_name, 1);
		stage = far_ready;
	}

	void medial_axis_t::set_close_vf_from_file(std::string file_name)
	{
		prx_assert(stage >= init_done, "Close VF not set.");
		vector_field_from_file(file_name, 0);

		// can't set stage to close_ready bc might not have far VF file
		// ==> need to set goal, sknw & prep graph
		stage = map_set;
	}

	void medial_axis_t::vector_field_from_file(std::string file_name, int map)
	{
		// prx_assert(exists(file_name), "File does not exists!");
   		std::ifstream ifs(file_name);
    	std::string line;

		int v1, v2;
		int h,w;
		int i = 0;
		while(std::getline(ifs,line))
		{
			i = h = w = v1 = v2 = 0 ;
            std::istringstream ss(line);
			std::string token;
			std::vector<int> p;
			while(std::getline(ss, token, ' ')) 
			{
				switch(i)
				{
					case 0: w  = std::stoi(token); break;
					case 1: h  = std::stoi(token); break;
					case 2: v1 = std::stoi(token); break;
					case 3: v2 = std::stoi(token); break;
					default: break;
				}

				i++;
			}
			if (map == 0) map_close(w,h) = std::complex<double>(v1, v2);
			else if (map == 1) map_far(w,h) = std::complex<double>(v1, v2);
        }
	
	}

	void medial_axis_t::edges_to_file(std::string file_name)
	{
		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);
		std::complex<double> uv;

		for (int i = 0; i <  rows(); ++i)
		{
			for (int j = 0; j <  cols(); ++j)
			{
				if ( is_edge(i, j) )
				{
					ofs_map << get_edge_id(i,j) << " " << i << " " << j << std::endl;
				}
			}
		}
		ofs_map.close();

	}

	void medial_axis_t::integrated_vf_to_file(std::string file_name)
	{
		std::cout << "Saving integrated VF to: " << file_name << std::endl;

		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		std::complex<double> uv;

		for (int i = 0; i < rows(); ++i)
		{
			for (int j = 0; j < cols(); ++j)
			{
				// if (is_node(i,j) || is_obstacle(i,j))
				// {
				uv = integrated_vector_at(i,j, 10, 5, true);
				// uv = integrated_vector_at(i,j, std::numeric_limits<double>::max(), false);
				// uv = compute_integrated_vector(i, j);
				ofs_map << i << " " << j << " ";
				ofs_map << uv.real() << " " << uv.imag() << std::endl;
				// }

			}
		}
		ofs_map.close();
	}

	void medial_axis_t::blossom_to_file(std::string file_name)
	{
		std::cout << "Saving blossom vectors to: " << file_name << std::endl;

		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		std::complex<double> uv;

		for (int i = 0; i < rows(); ++i)
		{
			for (int j = 0; j < cols(); ++j)
			{
				// if (is_node(i,j) || is_obstacle(i,j))
				// {
				// if (!is_obstacle(i,j) && uniform_random() < .8) continue;
				if (is_obstacle(i,j)) 
				{
					uv = integrated_vector_at(i,j, 100, 10, true);

					ofs_map << i << " " << j << " ";
					ofs_map << uv.real() << " " << uv.imag() << std::endl;
				}
				if (uniform_random() < .99) continue;
				auto vs = blossom(i, j, 5, 100, 10, true);
				prx_assert(vs.size() > 1 || is_obstacle(i,j) || !in_bounds(i,j), "returning only one vec:  ( " << i << ", " << j << " )");
				for (auto v : vs)
				{
					ofs_map << i << " " << j << " ";
					ofs_map << v.real() << " " << v.imag() << std::endl;
				}

			}
		}
		ofs_map.close();
	}

	void medial_axis_t::nodes_to_file(std::string file_name)
	{
		std::ofstream ofs_map;
		ofs_map.open(file_name.c_str(), std::ofstream::trunc);

		for (int i = 0; i < rows(); ++i)
		{
			for (int j = 0; j < cols(); ++j)
			{
				if ( is_node(i, j) )
				{
					ofs_map << get_node_id(i,j) << " " << i << " " << j << std::endl;
				}
			}
		}
		ofs_map.close();
	}

	void medial_axis_t::grap_vertices_to_file(std::string file_name)
	{
		prx_assert(stage >= graph_ready, "Graph has not been prepared.");
		graph.vertex_list_to_file(file_name);
	}

	void medial_axis_t::gnn_to_file(std::string file_name)
	{
		prx_assert(stage >= graph_ready, "Graph has not been prepared.");
		// metric_obstacles -> nodes_to_file(file_name);
	}

	bool medial_axis_t::has_direct_line_of_sight(space_point_t init, space_point_t end, double clearance)
	{
		double dist = df_ma(init,end);
		if (dist <= 0.01) return true;
		double min_dist, min_clearance;
		std::function<bool(space_point_t)> is_in_bounds = [&](space_point_t p1)
		{
			return 	p1 -> at(0) >= 0 &&
					p1 -> at(1) >= 0 &&
					p1 -> at(0) <  rows() &&
					p1 -> at(1) <  cols();
		};
		
		const double delta = 1.0 / (2.*dist);
		// if (init -> at(0) == 188 && init -> at(1) == 58) printf("delta: %.4f\n", delta);
		for (double i = delta; i <= 1; i+=delta)
		{
			state_space -> interpolate(init,end,i,pt_cl1);

    		if ( is_obstacle(std::floor(pt_cl1 -> at(0)), std::floor(pt_cl1 -> at(1))))
    		{
    			return false;
    		}

    		min_dist = std::min(dist * i, dist * (1. - i));
		// MA_DEBUG(std::floor(init -> at(0)), std::floor(init -> at(1)), 160, 160, "\tpt_cl1: " << state_space -> print_point(pt_cl1, 2) )

    		min_clearance = std::min(clearance, min_dist);//clearance < min_dist ? clearance : min_dist;
    		double dist_to_obst = dist_to_obstacle(std::complex<double>(std::floor(pt_cl1 -> at(0)), std::floor(pt_cl1 -> at(1))));
    		if (dist_to_obst < 1.) continue;

    		if (dist_to_obst < min_clearance) return false;

		}
		return true;
	}

	void medial_axis_t::gradient_vector_at(int i, int j)
	{
		auto iv = integrated_vector_at(i,j, 1.0);

	}

	void medial_axis_t::compute_distances_to_ma()
	{
		std::function<void(int, int, double)> check_and_add_to_queue = [&](int n, int m, double dist)
		{
			if (in_bounds(n, m) && ma_map[n][m].dist_to_ma > dist) 
			{
				ma_map[n][m].dist_to_ma = dist;
				ma_queue.push(std::make_pair(n, m));
			}
		};
		int i, j;
		double d;
		double sqrt2 = std::sqrt(2.0);
		while (ma_queue.size() > 0)
		{
			i = ma_queue.front().first;
			j = ma_queue.front().second;
			d = ma_map[i][j].dist_to_ma;
			check_and_add_to_queue(i-1, j-1, d + sqrt2);
			check_and_add_to_queue(i-1,  j , d + 1.0);
			check_and_add_to_queue(i-1, j+1, d + sqrt2);
			check_and_add_to_queue( i , j-1, d + 1.0);
			check_and_add_to_queue( i , j+1, d + 1.0);
			check_and_add_to_queue(i+1, j-1, d + sqrt2);
			check_and_add_to_queue(i+1,  j , d + 1.0);
			check_and_add_to_queue(i+1, j+1, d + sqrt2);
	
			ma_queue.pop();
		}
	}
}