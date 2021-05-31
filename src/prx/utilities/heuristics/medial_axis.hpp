#pragma once

#include <set>
#include <queue>
#include <vector>
#include <string>
#include <memory>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <chrono>

#include "prx/utilities/defs.hpp"
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/undirected_graph.hpp"
// #include "prx/utilities/data_structures/tree.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"

// #define z_obstacle std::complex<double>(std::numeric_limits<double>::max(),std::numeric_limits<double>::max())
#define MA_DEBUG(i,j,iL,jL,str) if (i == iL && j == jL) std::cout << str << std::endl;
#define EXPAND_CMP(cmp) cmp.real(), cmp.imag()

namespace prx
{
	typedef std::function<void(int,int,space_point_t)> mapping_f;
	typedef std::complex<double> ma_pt;
	/**
	 * Compute the medial axis and vector field for a 2D map
	 */
	// static std::function<bool(std::complex<double>,std::complex<double>)> cmpx_comp = [] (std::complex<double> a, std::complex<double> b)
	// 	{
	// 		return std::abs(a) < std::abs(b);
	// 	};

	struct cmpx_comp {
    	bool operator()(const std::complex<int> a, const std::complex<int> b) const 
    	{ 
        	return std::abs(a) < std::abs(b);
    	}
	};
	class ma_map_t;
	class medial_axis_t;

	class ma_cell
	{
	private:
		bool is_edge; 
		bool is_node;
		edge_index_t edge_id;
		node_index_t node_id;
		double dist_to_ma;
		double dist_to_obs;
		std::complex<double> close;
		std::complex<double> far;
		std::complex<double> integrated;
		std::complex<double> to_ma;
		int close_count;
		// static int next_edge_id;
		// static int next_node_id;

	public:
		static constexpr std::complex<double> z_obstacle = std::complex<double>(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
		ma_cell()
		{
			is_node = false;
			is_edge = false;
			close = std::complex<double>(0,0);
			far = std::complex<double>(0,0);
			integrated = std::complex<double>(0,0);
			to_ma = std::complex<double>(0,0);
			dist_to_obs = std::numeric_limits<double>::infinity();
			dist_to_ma = std::numeric_limits<double>::infinity();
			edge_id = -1;
			node_id = -1;
			close_count = 0;
		};
		~ma_cell() = default;
		bool is_obstacle()
		{
			return close == z_obstacle || far == z_obstacle || integrated == z_obstacle;
		}
		friend class medial_axis_t;
	};

	class medial_axis_t
	{
	private:
		/**
		 * The computation of the vector field has the following flowchart:
		 * 
		 * 						   Construction
		 * 								|
		 *         					   init
		 *         					    |
		 *          --------------------+---------------------------
		 *          |												|
		 * 		 set_map								set_close_vf_from_file
		 *   		|												|	
		 *   compute_close_VF 										|
		 *          |												|
		 *   		+------------+----------------------------------+
		 *   					 |									|
		 * 		   			 set_goal						set_far_vf_from_file
		 *          			 |									|
		 *          	+-----------------+							|
		 *          	|				  |							|
		 * 			set_sknw 		compute_sknw					|
		 *    			|				  |							|
		 *    			---------+---------							|
		 *    					 |									|
		 * 			   	   prepare_graph							|
		 *           		     |									|
		 *           	      	 |									|
		 *                    	 |				   					|
		 * 	               compute_far_VF	 						|
		 *  		 	         |				  					|
		 * 			 			 |				  					|	
		 *           			 +-----------+----------------------+
		 * 					            	 |
		 * 					     +-------(Queries)*
		 *           		     |			 |
		 *                 	     |			 |
		 * 					     |	  (Save to files)*
		 *        			     |			 |
		 *               	     +-----------+
		 * 
		 * Queries: integrated_vector_at, close_vector_at, far_vector_at
		 *
		 * Save to files: integrated_vf_to_file, close_vf_to_file, far_vf_to_file, grap_vertices_to_file, gnn_to_file
		 * 
		 */
		enum Stage { instatiated, init_done, map_set, close_ready, goal_set, sknw_set, graph_ready, far_ready};
		int next_edge_id = 0;
		int next_node_id = 0;
		space_t* state_space;
		// Eigen::Matrix <std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> close;
		// Eigen::Matrix <std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> map_close;
		// Eigen::Matrix <std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> map_far;
		// ma_map_t* ma_map;
		undirected_graph_t graph;
		undirected_graph_t obstacles_graph;
		// undirected_graph_t graph_obstacles;
		graph_nearest_neighbors_t* metric_close;
		graph_nearest_neighbors_t* metric_obstacles;
		std::vector<double*> state_memory;
		trajectory_t* aux_traj;
		std::complex<double> v_zero;//(0,0);
		std::complex<double> v_not_computed;//(0,0);
		// std::complex<double> z_obstacle;
		node_index_t goal_index;
		space_point_t goal;

		Stage stage;

		// Auxiliary variables for different functions
		double h,w;
		space_point_t pt;
		space_point_t pt_cl1, pt_cl2;
		// space_point_t space_point;
		// Eigen::Vector2d v1, v2, v3;
		ma_pt v1, v2, v3;

		std::string map_file_name;

		// Line of Sight clearance
		double los_clearance;

		int height;
		int width;
		std::vector<std::vector<ma_cell>> ma_map;
		std::queue<std::pair<int,int>> ma_queue;
		std::unordered_map<int, std::pair<int, int>> nodes_edges;

		// key: node_index
		// value: vector of edges_index
		std::unordered_map<int, std::vector<int>> edges_list;

		// std::random_device rd;
    	// std::mt19937 rd_mt19937(std::random_device());
		// ------------
		// | 7 | 0 | 1 |
		// | 6 | p | 2 |
		// | 5 | 4 | 3 |
		// ------------
		std::vector<std::pair<ma_pt,double>> dirs = {	
						std::pair<ma_pt,double>(std::complex<double>(+1,+0), std::abs(std::complex<double>(+1,+0))),
						std::pair<ma_pt,double>(std::complex<double>(+1,+1), std::abs(std::complex<double>(+1,+1))),
						std::pair<ma_pt,double>(std::complex<double>(+0,+1), std::abs(std::complex<double>(+0,+1))),
						std::pair<ma_pt,double>(std::complex<double>(-1,+1), std::abs(std::complex<double>(-1,+1))),
						std::pair<ma_pt,double>(std::complex<double>(-1,+0), std::abs(std::complex<double>(-1,+0))),
						std::pair<ma_pt,double>(std::complex<double>(-1,-1), std::abs(std::complex<double>(-1,-1))),
						std::pair<ma_pt,double>(std::complex<double>(+0,-1), std::abs(std::complex<double>(+0,-1))),
						std::pair<ma_pt,double>(std::complex<double>(+1,-1), std::abs(std::complex<double>(+1,-1)))
					};
		
		std::vector<std::pair<ma_pt,double>> dir4 = {	
						std::pair<ma_pt,double>(std::complex<double>(+1,+0), std::abs(std::complex<double>(+1,+0))),
						std::pair<ma_pt,double>(std::complex<double>(+0,+1), std::abs(std::complex<double>(+0,+1))),
						std::pair<ma_pt,double>(std::complex<double>(-1,+0), std::abs(std::complex<double>(-1,+0))),
						std::pair<ma_pt,double>(std::complex<double>(+0,-1), std::abs(std::complex<double>(+0,-1))),
					};
		// std::set<std::complex<int>, cmpx_comp> edges_set;
		// ma_map related functions
		void init_ma_map()
		{
			for (int i = 0; i < height; ++i)
			{	
				std::vector<ma_cell> row;
				for (int j = 0; j < width; ++j)
				{
					row.push_back(ma_cell());
				}
				ma_map.push_back(row);
			}
		}

		void set_obstacle(int i, int j, bool obst)
		{
			if (obst)
			{
				ma_map[i][j].far = ma_cell::z_obstacle;
				ma_map[i][j].close = ma_cell::z_obstacle;
				ma_map[i][j].integrated = ma_cell::z_obstacle;
			}
			else
			{
				ma_map[i][j].far = v_not_computed;
				ma_map[i][j].close = v_not_computed;
				ma_map[i][j].integrated = v_not_computed;	
			}
		}

		std::complex<double>& map_far(int i, int j)
		{
			return ma_map[i][j].far;
		}
		const std::complex<double> map_far(int i, int j) const
		{
			return ma_map[i][j].far;
		}

		std::complex<double>& map_close(int i, int j)
		{
			return ma_map[i][j].close;
		}
		const std::complex<double> map_close(int i, int j) const
		{
			return ma_map[i][j].close;
		}
		std::complex<double>& to_ma(int i, int j)
		{
			// printf("(%d, %d)\n", i, j);
			i = std::max(0, std::min(i, height - 1));
			j = std::max(0, std::min(j, width  - 1));
			return ma_map[i][j].to_ma;
		}
		const std::complex<double> to_ma(int i, int j) const
		{
			i = std::max(0, std::min(i, height - 1));
			j = std::max(0, std::min(j, width  - 1));
			return ma_map[i][j].to_ma;
		}

		inline int rows(){return height;}
		inline int cols(){return width;}

		void set_as_edge(int i, int j)
		{
			// if (ma_map[i][j].close == ma_cell::z_obstacle) return;
			// if (ma_map[i][j].is_node) return;
			ma_map[i][j].is_edge = true;
			// ma_map[i][j].dist_to_ma = 0;
		}

		void set_as_edge(int i, int j, int edge_id)
		{
			if (ma_map[i][j].close == ma_cell::z_obstacle) return;
			if (ma_map[i][j].is_node) return;
			ma_map[i][j].is_edge = true;
			ma_map[i][j].dist_to_ma = 0;
			ma_map[i][j].edge_id = edge_id;
			next_edge_id = std::max(next_edge_id, edge_id+1);
			ma_queue.push(std::make_pair(i,j));

		}

		void unset_edge(int i, int j)
		{
			ma_map[i][j].is_edge = false;
			ma_map[i][j].edge_id = -1;

		}

		double get_dist_to_ma(int i, int j)
		{
			return ma_map[i][j].dist_to_ma;
		}

		inline void set_as_node(ma_pt v, int node_id)
		{
			set_as_node(v.real(), v.imag(), node_id);
		}
		inline void set_as_node(int i, int j, int node_id)
		{
			i = std::max(0, std::min(i, height - 1));
			j = std::max(0, std::min(j, width  - 1));
			ma_map[i][j].is_node = true;
			ma_map[i][j].dist_to_ma = 0;
			ma_map[i][j].node_id = node_id;
			next_node_id = std::max(next_node_id, node_id+1);
			ma_queue.push(std::make_pair(i,j));
		}
		inline void set_node_as(int i, int j, bool val)
		{
			ma_map[i][j].is_node = val;
		}
		// bool surrounded_by_obstacles(int i, int j);
		
		inline bool is_edge(std::complex<double> v)
		{
			return is_edge(v.real(), v.imag());
		}
		inline bool is_edge(int i, int j)
		{
			if (i < 0 || i >= rows()) return false;
			if (j < 0 || j >= cols()) return false;
			return ma_map[i][j].is_edge;
		}

		int get_edge_id(int i, int j)
		{
			return ma_map[i][j].edge_id;
		}

		bool is_node(ma_pt v)
		{
			if (v.real() < 0 || v.real() >= rows()) return false;
			if (v.imag() < 0 || v.imag() >= cols()) return false;
			
			return  ma_map[v.real()][v.imag()].is_node;
		}
		bool is_node(int i, int j)
		{
			if (i < 0 || i >= rows()) return false;
			if (j < 0 || j >= cols()) return false;
			return ma_map[i][j].is_node;
		}

		bool is_goal(double i, double j)
		{
			if (i < 0 || i >= rows()) return false;
			if (j < 0 || j >= cols()) return false;
			auto id1 = ma_map[std::floor(i)][std::floor(j)].node_id;
			auto id2 = ma_map[std::ceil(i)][std::ceil(j)].node_id;
			if (id1 == goal_index || id2 == goal_index) printf("GOAL!\n");
			return id1 == goal_index || id2 == goal_index;			
		}

		inline node_index_t get_node_id(int i, int j)
		{
			return ma_map[i][j].node_id;
		}
		inline edge_index_t get_node_id(ma_pt v)
		{
			return get_node_id(v.real(), v.imag());
		}
		inline const double dist_to_obstacle(ma_pt v) const
		{
			return ma_map[v.real()][v.imag()].dist_to_obs;
		}
		inline double& dist_to_obstacle(ma_pt v)
		{
			return ma_map[v.real()][v.imag()].dist_to_obs;
		}
		inline bool is_obstacle(int i, int j)
		{
			// Lets say the bounds are obstacles...
			// if (i < 0 || i >= rows()) return true;
			// if (j < 0 || j >= cols()) return true;
			return 	(i < 0 || i >= rows()) ||
			 		(j < 0 || j >= cols()) || 
			 		ma_map[i][j].is_obstacle();
		}

		inline bool in_bounds(int i, int j)
		{
			return 0 <= i && 0 <= j && i <  cols() && j <  rows();
		}

		void compute_distances_to_ma();

		/**
		 * Check if there are no obstacles in a straigh line from init to end
		 * @param  init Point where to start the line segment
		 * @param  end  Point where the line segment ends
		 * @return      True if no obstacles are found, false otherwise
		 */
		// bool has_direct_line_of_sight(space_point_t init, space_point_t end);
		bool has_direct_line_of_sight(space_point_t init, space_point_t end, double clearance = 0.0);

		void add_goal_to_graph();

		// read a vector field from file. 
		// If close_far == 0 ==> close
		// If close_far == 1 ==> far
		// Each line of the file has to be in the form:
		// i j u v
		// Where map(i,j) = (u,v).
		void vector_field_from_file(std::string file_name, int close_far);

		void add_obstacles_to_graph();

	public:	
		// valid_trajectory_t valid_check;
		//distance function to the medial axis
		distance_function_t df_ma; 
		// distance function: pixel to medial axis to goal
		medial_axis_t()
		{	
			// valid_check = nullptr;
			df_ma = nullptr;
			goal = nullptr;
			v_zero = std::complex<double>(0,0);
			v_not_computed = std::complex<double>(std::numeric_limits<double>::min(),std::numeric_limits<double>::min());
     		

			state_memory = {&h, &w};
			state_space = new space_t("EE",state_memory,"HW");
			state_space -> set_bounds(	{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()},
										{ std::numeric_limits<double>::max(),  std::numeric_limits<double>::max()});

			aux_traj = new trajectory_t(state_space);
    		df_ma = [](const space_point_t& p1, const space_point_t& p2)
			{
				double cost = 0;
				for (int i = 0; i < p1 -> get_dim(); ++i)
				{
					cost += std::pow(p1 -> at(i) - p2 -> at(i), 2);
				}
				return std::sqrt(cost);
			};
			// pt =  state_space -> make_point();
			pt =  state_space -> make_point();
			pt_cl1 =  state_space -> make_point();
			pt_cl2 =  state_space -> make_point();

			los_clearance = 5;
			stage = instatiated;

		};

		void init()
		{	
			prx_assert(df_ma != nullptr, "distance_function not set for medial_axis_t!");
			metric_close = new graph_nearest_neighbors_t(df_ma);
			metric_obstacles = new graph_nearest_neighbors_t(df_ma);
			graph.clear();
			graph.allocate_memory<undirected_node_t,undirected_edge_t>(1000);
			obstacles_graph.clear();
			obstacles_graph.allocate_memory<undirected_node_t,undirected_edge_t>(1000);
			metric_close -> clear();
			metric_obstacles -> clear();
			stage = init_done;
		}

		void set_goal(std::vector<double> _goal)
		{
			prx_assert(stage >= close_ready, "Close VF has not been set/computed.");

			goal = state_space -> make_point();
			state_space -> copy_point_from_vector(goal, _goal);
			// std::cout << state_space -> print_point(goal, 2) << std::endl;
			stage = goal_set;
		}

		void set_map(const std::string map_file);

		void set_sknw(const std::string nodes_file, const std::string sknw_file, const std::string edges_file); // Should get skeletonization within c++

		void cost_to_go_to_file(std::string file_name);

		void integrated_vf_to_file(std::string file_name);

		void close_vf_to_file(std::string file_name, bool normalized = true);

		void far_vf_to_file(std::string file_name, bool normalized = true);

		void grap_vertices_to_file(std::string file_name);

		void gnn_to_file(std::string file_name);

		void set_close_vf_from_file(std::string file_name);
	
		void set_far_vf_from_file(std::string file_name);

		void prepare_graph();

		/**
		 * Compute the close vector at (i, j) 
		 * @param i Compute the element of the ith row
		 * @param j Compute the element of the jth column
		 */
		void compute_close_vector(int i, int j);

		/**
		 * Compute the far vector at (i, j) 
		 * @param i Compute the element of the ith row
		 * @param j Compute the element of the jth column
		 */
		void compute_far_vector(int i, int j);

		/**
		 * Compute the full close vector field
		 */
		void compute_close_vector_field();

		/**
		 * Compute the full far vector field
		 */
		void compute_far_vector_field();

		/**
		 * Compute both vector fields at the same time. 
		 * This method is more efficient than computing 
		 * both vector fields by they own.
		 */
		void compute_vector_fields();

		void edges_to_file(std::string file_name);

		void nodes_to_file(std::string file_name);
		
		std::complex<double> compute_integrated_vector(int i, int j);
		
		std::vector<std::complex<double>> blossom(int i, int j, int n, double max_magnitud = std::numeric_limits<double>::max(), double min_magnitud = 0.0, bool avg_if_obstacle = false);

		/**
		 * Compute the integrated vector at (i,j)
		 * The magnitud m of the output vector is such that: m \in [min_magnitud, max_magnitud]
		 * The default values are [0, \inf]
		 * Notice that if 0 < min_magnitud the resulting vector could be in collision.
		 */
		std::complex<double> integrated_vector_at(int i, int j, double max_magnitud = std::numeric_limits<double>::max(), double min_magnitud = 0.0, bool avg_if_obstacle = false);

		std::complex<double> close_vector_at(int i, int j, bool normalized = true);

		std::complex<double> far_vector_at(int i, int j, bool normalized = true);

		void blossom_to_file(std::string file_name);

		void compute_sknw_and_save_to_files(std::string nodes_file, std::string edges_file, std::string sknw_file,
					std::string py_sknw = lib_path + "/scripts/medial_axis.py", std::string py_cmd = "python3");

		void gradient_vector_at(int i, int j);

		// @Aravind: I had to include this line to get this file to compile.
		typedef std::function<void (int,int,space_point_t)> mapping_f;

		void set_map(int x_grid, int y_grid, valid_state_t& valid_state, mapping_f f,
			space_t* state_space, std::string _map_name = lib_path + "/out/ma_custom.map");

		void find_medial_axis();

		double cost_to_go(int i, int j);




	};
}
