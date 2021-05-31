#pragma once

#include "prx/utilities/spaces/space.hpp"
#include "prx/planning/world_model.hpp"
#include <vector>

namespace prx
{
	class node_t
	{
	public:

		node_t(const int x1, const int y1, const double cost)
		{
			x = x1;
			y = y1;
			g = cost;
		}

		virtual ~node_t() { }

		bool operator <(const node_t other) const
		{
			return g > other.g;
		}

		bool operator ==(const node_t other) const
		{
			return x == other.x && y == other.y;
		}

		bool operator !=(const node_t other) const
		{
			return x != other.x || y != other.y;
		}

		double g;
		int x;
		int y;
		node_t *neighbors;
	};

	class seen_node_t
	{
	public:

		seen_node_t(node_t * _n)
		{
			n = _n;
		}

		virtual ~seen_node_t() { }

		bool operator ==(const seen_node_t other) const
		{
			return n == other.n;
		}

		bool operator !=(const seen_node_t other) const
		{
			return n != other.n;
		}

		node_t *n;
	};

	class heuristic_map_t
	{
	public:

		heuristic_map_t(double origin[2], double goal_pos[2], double c_size);
		heuristic_map_t(const double c_size);
		// iai constructor
		heuristic_map_t (int init_grid_length=10, int init_grid_with=10, double flag_PRX_UNSEEN_CELL=2.0);

		heuristic_map_t(const double world_origin[2],
				const double _goalx,
				const double _goaly,
				const int grid_h,
				const int grid_w,
				const double c_size,
				const int _k,
				const bool use_min_cost1
		);

		// corl constructor
		heuristic_map_t(const double world_origin_x,
				const double world_origin_y,
				const double _goalx,
				const double _goaly,
				const int grid_h,
				const int grid_w,
				const double c_size,
				const std::string map_fname,
				const int padding
		);

		heuristic_map_t(const double world_origin_x,    // workspace coord
                const double world_origin_y,    // workspace coord
                const double _goalx,    // workspace coord
                const double _goaly,    // workspace coord
                const int grid_h,   // length (x)
                const int grid_w,   // (y)
                const double c_size, // resolution
                const std::vector<std::vector<bool>> extracted_map
        );

		virtual ~heuristic_map_t();

		void replan(const double world_origin[2], int dim[2], double start[2], double p_density=0.05);

		void global_to_local_map_space(int out[2], double global[2], int dim[2], double dx, double dy);

		void local_map_to_global_space(double out[2], int local[2], int dim[2], double dx, double dy);

		void global_to_global_map_space(int out[2], double global[2]);

		void global_map_to_global_space(double out[2], int global_map[2]);

		void local_map_to_global_map_space(int out[2], int local[2], int dim[2], double dx, double dy);

		double add_time_multiplier(double cost, double x, double y);

		bool in_grid(int, int);

		double **global_cost_map;
		double **global_h_map;
		bool ** seen_map;
		int goalx, goaly;
		double delta_x, delta_y, cell_size;
		int grid_length, grid_width, knn;
		bool use_min_cost;

		int min_x_coord=-1, max_x_coord=-1, min_y_coord=-1, max_y_coord=-1;

		double get_cost(const double x, const double y);
		double get_cost(const int x, const int y);

		heuristic_map_t(const double c_size, bool _use_obs_h);
		bool use_learning, use_obs_h;

		void init_env(std::vector<std::shared_ptr<movable_object_t>>&, std::vector<std::string>&);

		void plan_wavefront(const double _goalx, const double _goaly);

		double get_score(const space_point_t point);
		double get_score(const double x, const double y);
		double get_score(const int x, const int y);
		bool is_collision_free(const space_point_t point);

		void process_map(double ** new_map, double origin[2], int grid_h, int grid_w, const double _goalx, const double _goaly, double c_size);
		// double ** get_local_map(int center_map[2], int x_size, int y_size);
		std::vector<double> get_local_map(double _x, double _y, int size=41);
		std::vector<int> get_local_obs_map(double _x, double _y, int size=41);
		void free_map(double ** g, int grid_h);
		void print_cost_map();
		void print_h_map();
		std::vector<std::vector<std::vector<int>>> compute_vector_field();
	private:

		double PRX_UNSEEN_CELL = 2.0;

		double cell_round(double a);

		void free_map(bool ** g, int grid_h);

		double ** init_rand_map(double _start[2], int dim[2], double dx, double dy, double p_density=0.05);

		void add_to_global_map(double ** local_cost_map, int dim[2], double dx, int dy);

		void dijkstra();

		double ** create_cost_to_go_map();
		void create_cost_to_go_map(double ** g);

		int get_num_obs_neighbors(int x, int y);

		std::vector<node_t> get_neighbors(int _x, int _y);

		std::vector<node_t> get_neighbors(int x, int y, double ** g);

		double sum_parent_cost(int x, int y, double ** g, int *** p);

		double euclidean_distance(double x, double y);

		std::vector<node_t> insertion_sort(std::vector<node_t> neighbors, int x, int y);

		void get_k_nearest_neighbors(std::vector<node_t> out);

		bool line_of_sight(int x0, int y0, int x1, int y1);

	};
}
