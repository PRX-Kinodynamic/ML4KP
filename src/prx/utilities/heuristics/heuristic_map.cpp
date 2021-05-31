
#include "prx/utilities/heuristics/heuristic_map.hpp"
#include "prx/simulation/plants/cell_plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <random>
#include <ctime>
#include <queue>

#include <fstream>
namespace prx
{
	heuristic_map_t::heuristic_map_t(const double world_origin[2],
					const double _goalx,
					const double _goaly,
					const int grid_h,
					const int grid_w,
					const double c_size,
					const int _knn,
					const bool use_min_cost1
	)
	{
		grid_length = grid_h;
		grid_width = grid_w;
		cell_size = c_size;
		use_min_cost = use_min_cost1;
		use_learning = false;
		use_obs_h=false;
		knn = _knn;
		delta_x = world_origin[0] - grid_length / 2.0;
		delta_y = world_origin[1] - grid_width / 2.0;
		double global_goal[2] = {_goalx, _goaly};
		int goal_coords[2];
		global_to_global_map_space(goal_coords, global_goal);
		goalx = goal_coords[0];
		goaly = goal_coords[1];
		global_cost_map = (double **) malloc(grid_length * sizeof(double *));
		global_h_map = (double **) malloc(grid_length * sizeof(double *));
		seen_map = (bool **) malloc(grid_length * sizeof(bool *));
		for(int i = 0; i < grid_length; i++)
		{
			global_cost_map[i] = (double *) malloc(grid_width * sizeof(double));
			global_h_map[i] = (double *) malloc(grid_width * sizeof(double));
			seen_map[i] = (bool *) malloc(grid_width * sizeof(bool));
			for(int j = 0; j < grid_width; j++)
			{
				global_cost_map[i][j] = 0;
				seen_map[i][j] = false;

				global_h_map[i][j] = 0;
			}
		}
		for(int i = 0; i < grid_length; i++)
		{
			for(int j = 0; j < grid_width; j++)
			{
				if (i == 0  || i == grid_length-1 || j == 0  || j == grid_length-1)
				{
					std::srand(54321);
					std::default_random_engine generator;
					std::uniform_real_distribution<double> distribution(0.25 * std::min(grid_width, grid_length), std::min(grid_width, grid_length));
					global_cost_map[i][j] = 1;
					seen_map[i][j] = true;
					int obs_area_length = distribution(generator);
					int obs_area_width = obs_area_length;
					int mu = 0;
					// Fill buckets
					int x_min = std::max(0, i - obs_area_length/2), x_max = std::min(grid_length-1, i + obs_area_length/2);
					for (int x = x_min; x <= x_max; x++)
					{
						int y_min = std::max(0, j - obs_area_width/2), y_max = std::min(grid_width-1, j + obs_area_width/2);
						for (int y = y_min; y <= y_max; y++)
						{
							if (x == i && y == j)
								continue;
							double sigma = 2./cell_size;
							double d = sqrt((i-x)*(i-x) + (j-y)*(j-y));
							// seen_map[x][y] = true;
							global_cost_map[x][y] = std::max(global_cost_map[x][y], global_cost_map[i][j] * exp( -((d-mu)*(d-mu) / (2.0 * sigma*sigma)) ) );
						}
					}
				}
			}
		}
	}

	heuristic_map_t::heuristic_map_t(const double c_size)
	{
		// grid_length = 22.2 / c_size + 22; // added extra space for local map collection - 21 extra cells in each direction for 41x41 local map
		// grid_width = 22.2 / c_size + 22;
		grid_length = 22.2 / c_size + 51; // added extra space for local map collection - 21 extra cells in each direction for 41x41 local map
		grid_width = 22.2 / c_size + 51;
		delta_x = -(grid_length-1) / 2.0;
		delta_y = -(grid_width-1) / 2.0;
		cell_size = c_size;
		use_min_cost = true;
		use_learning = false;
		use_obs_h = true;
		global_cost_map = (double **) malloc(grid_length * sizeof(double *));
		global_h_map = (double **) malloc(grid_length * sizeof(double *));
		seen_map = (bool **) malloc(grid_length * sizeof(bool *));
		for(int i = 0; i < grid_length; i++)
		{
			global_cost_map[i] = (double *) malloc(grid_width * sizeof(double));
			global_h_map[i] = (double *) malloc(grid_width * sizeof(double));
			seen_map[i] = (bool *) malloc(grid_width * sizeof(bool));
			for(int j = 0; j < grid_width; j++)
			{
				global_cost_map[i][j] = 0;
				seen_map[i][j] = false;

				global_h_map[i][j] = 0;
			}
		}
	}

	// IAI constructor
	heuristic_map_t::heuristic_map_t(int init_grid_length, int init_grid_with, double flag_PRX_UNSEEN_CELL)
	{
		use_min_cost = true;
		use_learning = false;
		use_obs_h = false;
		knn = 5;
		grid_length = init_grid_length;
		grid_width = init_grid_with;
		delta_x = -0.420420420420;
		delta_y = -1;
		PRX_UNSEEN_CELL = flag_PRX_UNSEEN_CELL;

		global_cost_map = (double **) malloc(grid_length * sizeof(double *));
		global_h_map = (double **) malloc(grid_length * sizeof(double *));
		seen_map = (bool **) malloc(grid_length * sizeof(bool *));
		for(int i = 0; i < grid_length; i++)
		{
			global_cost_map[i] = (double *) malloc(grid_width * sizeof(double));
			global_h_map[i] = (double *) malloc(grid_width * sizeof(double));
			seen_map[i] = (bool *) malloc(grid_width * sizeof(bool));
			for(int j = 0; j < grid_width; j++)
			{
				global_cost_map[i][j] = -1;
				seen_map[i][j] = false;
				global_h_map[i][j] = 0;
			}
		}
	};
	/*
	heuristic_map_t::heuristic_map_t(double origin[2], double goal_pos[2], double c_size)
	{
		grid_length = 100;
		grid_width = 100;
		// grid_length = grid_l;
		// grid_width = grid_w;
		cell_size = c_size;
		use_min_cost = true;
		use_learning = false;
		use_obs_h=false;
		knn = 5;
		delta_x = origin[0] - grid_length / 2.0;
		delta_y = origin[1] - grid_width / 2.0;
		int goal_coords[2];
		global_to_global_map_space(goal_coords, goal_pos);
		goalx = goal_coords[0];
		goaly = goal_coords[1];
		global_cost_map = (double **) malloc(grid_length * sizeof(double *));
		global_h_map = (double **) malloc(grid_length * sizeof(double *));
		seen_map = (bool **) malloc(grid_length * sizeof(bool *));
		for(int i = 0; i < grid_length; i++)
		{
			global_cost_map[i] = (double *) malloc(grid_width * sizeof(double));
			global_h_map[i] = (double *) malloc(grid_width * sizeof(double));
			seen_map[i] = (bool *) malloc(grid_width * sizeof(bool));
			for(int j = 0; j < grid_width; j++)
			{
				global_cost_map[i][j] = -1;
				seen_map[i][j] = false;
				global_h_map[i][j] = 0;
			}
		}
	}
	*/

	void heuristic_map_t::process_map(double ** new_map, double origin[2], int map_l, int map_w, const double _goalx, const double _goaly, double c_size)
	{
		if (delta_x == -0.420420420420)
		{
			cell_size = c_size;	// Cell size expected to be static
			delta_x = origin[0] - grid_length / 2.0;
			delta_y = origin[1] - grid_width / 2.0;
		}
		// Add new map to global map if a new one has been passed
		if (map_l != grid_length || map_w != grid_width)
		{
			std::cout << "[heuristic_map] expanding grid"<<std::endl;
			double ** new_global_cost_map = (double **) malloc(map_l * sizeof(double *));
			double ** new_global_h_map = (double **) malloc(map_l * sizeof(double *));
			bool ** new_seen_map = (bool **) malloc(map_l * sizeof(bool *));
			for(int i = 0; i < map_l; i++)
			{
				new_global_cost_map[i] = (double *) malloc(map_w * sizeof(double));
				new_global_h_map[i] = (double *) malloc(map_w * sizeof(double));
				new_seen_map[i] = (bool *) malloc(map_w * sizeof(bool));
				for(int j = 0; j < map_w; j++)
				{
					if (new_map[i][j] == PRX_UNSEEN_CELL) new_global_cost_map[i][j] = new_map[i][j];
					else new_global_cost_map[i][j] = new_map[i][j];
					new_seen_map[i][j] = false;
					new_global_h_map[i][j] = 0;
				}
			}
			// add old information
			for(int i = 0; i < grid_length; i++)
			{
				for (int j = 0; j < grid_width; j++)
				{
					int x = i + grid_length;
					int y = j + grid_width;
					new_seen_map[x][y] = seen_map[i][j];
					new_global_h_map[x][y] = global_h_map[i][j];
				}
			}
			// print_cost_map();
			// std::cout<<std::endl;
			// free old maps
			free_map(global_cost_map, grid_length);
			free_map(global_h_map, grid_length);
			free_map(seen_map, grid_length);
			global_cost_map = new_global_cost_map;
			seen_map = new_seen_map;
			double orig_x = delta_x + grid_length/2.0;
			double orig_y = delta_y + grid_width/2.0;


			global_h_map = new_global_h_map;
			grid_length = map_l;
			grid_width = map_w;

			delta_x = orig_x - (map_l) / 2.0;
			delta_y = orig_y - (map_w) / 2.0;
			std::cout << "delta_x,y: "<<delta_x<<"\t"<<delta_y<<std::endl;
			std::cout <<"orig coords: "<< orig_x<<"\t"<<orig_y<<std::endl;
			std::cout<<"Unseen cell: "<<PRX_UNSEEN_CELL<<std::endl;
			// recalculate with new grid length and width
			// dl = ((origin[0]-(map_l/2.0*cell_size)) - delta_x);		// (X_local_bleft-X_global_bleft)*cell_size
			// dw = ((origin[1]-(map_w/2.0*cell_size)) - delta_y);
			int dl = (origin[0] - (delta_x+grid_length / 2.0)) * cell_size + grid_length/2.0 - map_l/2.0;		// (X_local_bleft-X_global_bleft)*cell_size
			int dw = (origin[1] - (delta_y+grid_width / 2.0)) * cell_size + grid_width/2.0 - map_w/2.0;
			std::cout<<"bleft local: "<<(origin[0]-(map_l/2.0*cell_size))<<"\t"<<(origin[1]-(map_w/2.0*cell_size))<<std::endl;
			std::cout<<"New dl, dw: "<<dl<<"\t"<<dw<<std::endl;
			// print_cost_map();
		}
		// dl = (origin[0] - (delta_x+grid_length / 2.0)) * cell_size + grid_length/2.0 - map_l/2.0*cell_size;		// (X_local_bleft-X_global_bleft)*cell_size
		// dw = (origin[1] - (delta_y+grid_width / 2.0)) * cell_size + grid_width/2.0 - map_w/2.0*cell_size;
		else
		{
			// Add information from new map
			for (int i = 0; i < map_l; i++)
			{
				for (int j = 0; j < map_w; j++)
				{
					if (new_map[i][j] == PRX_UNSEEN_CELL) global_cost_map[i][j] = new_map[i][j];
					else global_cost_map[i][j] = new_map[i][j];
					// global_cost_map[i][j] = new_map[i][j] > 1 ? 1 : new_map[i][j];
					// global_cost_map[i][j] = new_map[i][j] > 1 ? 1 : 1 - new_map[i][j];
				}
			}
		}

		double global[2] = {_goalx, _goaly};
		int g_map[2];
		global_to_global_map_space(g_map, global);
		goalx = g_map[0];
		goaly = g_map[1];
		std::cout<<"goal map coords: "<<goalx<<"\t"<<goaly<<std::endl;

		// double max_val=1.;
	        // for(int x=0;x<grid_length;x++)
	        // {
	        //         for(int y=0;y<grid_width;y++)
	        //         {
	        //                 // std::cout << get_score(x,y) << "\t";
	        //                 max_val = max_val > get_score(x,y) || get_score(x,y) == std::numeric_limits<double>::max() ? max_val : get_score(x,y);
	        //                 // max_val = max_val > global_cost_map[x][y] || global_cost_map[x][y] == std::numeric_limits<double>::max() ? max_val : global_cost_map[x][y];
	        //         }
	        //         // std::cout << std::endl;
	        // }
	        // std::cout << max_val << std::endl;
		// std::ofstream outfile;
	        // std::string map_string = "P2\n"+std::to_string(grid_length)+" "+std::to_string(grid_width)+"\n"+std::to_string((int)max_val)+"\n";
	        // for (int j = grid_width-1; j >= 0; j--)
	        // {
		// 	// for(int j = 0; j < grid_width; j++)
		// 	for (int i = 0; i < grid_length; i++)
	        //         {
	        //                 double d_val = global_h_map[i][j];
	        //                 // double d_val = global_cost_map[i][j];
		// 		// if (d_val == PRX_UNSEEN_CELL) d_val = 1;
	        //                 int val = d_val == std::numeric_limits<double>::max() ? 0 : (int)max_val-d_val;
	        //                 map_string += std::to_string(val);
	        //                 if (i != grid_length-1) map_string += " ";
	        //         }
	        //         map_string += "\n";
	        // }
	        // outfile.open("/home/milkkarten/test1.pgm");
	        // outfile << map_string;
	        // outfile.close();
	        // std::cout<<"Obstacle map successfully saved at /home/milkkarten/test1.pgm"<<std::endl;

		dijkstra();
		// print_h_map();
	}

	// double ** heuristic_map_t::get_local_map(int center_map[2], int x_size, int y_size)
	// {
	// 	int min_x = center_map[0] - x_size/2.0;
	// 	int min_y = center_map[1] - y_size/2.0;
	// 	double ** out = (double **) malloc(sizeof(double *)*x_size);
	// 	for (int i = 0; i < x_size; i++)
	// 	{
	// 		out[i] = (double *) malloc(sizeof(double)*y_size);
	// 		for (int j = 0; j < y_size; j++)
	// 		{
	// 			out[i][j] = global_cost_map[min_x+i][min_y+j];
	// 		}
	// 	}
	// 	return out;
	// }

	void heuristic_map_t::init_env(std::vector<std::shared_ptr<movable_object_t>>& obstacle_list, std::vector<std::string>& obstacle_names)
	{
		system_ptr_t plant = create_system<cell_plant_t>("cell");
		auto cell = std::dynamic_pointer_cast<cell_plant_t>(plant);
		cell->set_geo(0.005,0.005);
		world_model_t<> world_model({plant}, {obstacle_list});
		world_model.create_context("grid_context",{"cell"},{obstacle_names});
		auto context = world_model.get_context("grid_context");
		trajectory_t traj(context.first->get_state_space());
		plan_t plan(context.first->get_control_space());
		double mu = 0;
		for(int i = 0; i < grid_length; i++)
		{
			for(int j = 0; j < grid_width; j++)
			{
				double global[2];
				int g_map[2] = {i,j};
				global_map_to_global_space(global, g_map);
				space_point_t s = context.first->get_state_space()->make_point();
				std::vector<double> v;
				v.push_back(global[0]);
				v.push_back(global[1]);
				int dimension = context.first->get_state_space()->get_dimension();
				while (v.size() < dimension)
					v.push_back(0.);
				context.first->get_state_space()->copy_point_from_vector(s, v);
				context.first->get_state_space()->copy_from_point(s);
				// Collision check these coords
				if (context.second->in_collision())
				{
					global_cost_map[i][j] = 1.0;
					continue; // no obstacle smoothing
					int obs_area_length = 2;//1 / cell_size;	// 2 cells around obstacles
					int obs_area_width = obs_area_length;
					// Fill buckets
					int x_min = std::round(std::max(0, i - obs_area_length/2)), x_max = std::round(std::min(grid_length-1, i + obs_area_length/2));
					for (int x = x_min; x <= x_max; x++)
					{
						int y_min = std::round(std::max(0, j - obs_area_width/2)), y_max = std::round(std::min(grid_width-1, j + obs_area_width/2));
						for (int y = y_min; y <= y_max; y++)
						{
							if (x == i && y == j)
								continue;
							double sigma = 2./cell_size;
							double d = sqrt((i-x)*(i-x) + (j-y)*(j-y));
							global_cost_map[x][y] = std::max(global_cost_map[x][y], exp( -((d-mu)*(d-mu) / (2.0 * sigma*sigma)) ) );
						}
					}
				}
			}
		}
		// Find boundaries and make all areas outside boundaries obstacles
		int minx = grid_length-1, miny = grid_width-1, maxx=0, maxy=0;
		for(int i = 0; i < grid_length; i++)
		{
			for(int j = 0; j < grid_width; j++)
			{
				if (global_cost_map[i][j] == 1)
				{
					minx = i < minx ? i : minx;
					miny = j < miny ? j : miny;
					maxx = i > maxx ? i : maxx;
					maxy = j > maxy ? j : maxy;
				}
			}
		}
		for(int i = 0; i < grid_length; i++)
		{
			for(int j = 0; j < grid_width; j++)
			{
				if(i < minx || i > maxx || j < miny || j > maxy) global_cost_map[i][j] = 1;
			}
		}
	}
	void heuristic_map_t::plan_wavefront(const double _goalx, const double _goaly)
	{
		double global[2] = {_goalx, _goaly};
		int g_map[2];
		global_to_global_map_space(g_map, global);
		goalx = g_map[0];
		goaly = g_map[1];
		// std::cout <<goalx<<"\t"<<goaly<<std::endl;
		dijkstra();
	}
	double heuristic_map_t::get_score(const space_point_t point)
	{
		double global[2] = {point->at(0),point->at(1)};
		int g_map[2];
		global_to_global_map_space(g_map, global);
		if (g_map[0] < grid_length && g_map[1] < grid_width && g_map[0] >= 0 && g_map[1] >=0)
		{
			if (global_cost_map[g_map[0]][g_map[1]] == -1) return std::numeric_limits<double>::max();
			// if (global_cost_map[g_map[0]][g_map[1]] == 1)
			// 	return std::numeric_limits<double>::max();
			return global_h_map[g_map[0]][g_map[1]];
		}
		return std::numeric_limits<double>::max();

	}
	double heuristic_map_t::get_score(const double x, const double y)
	{
		double global[2] = {x, y};
		int g_map[2];
		global_to_global_map_space(g_map, global);
		if (g_map[0] < grid_length && g_map[1] < grid_width && g_map[0] >= 0 && g_map[1] >=0)
		{
			if (global_cost_map[g_map[0]][g_map[1]] == -1) return std::numeric_limits<double>::max();
			// if (global_cost_map[g_map[0]][g_map[1]] == 1)
			// 	return std::numeric_limits<double>::max();
			return global_h_map[g_map[0]][g_map[1]];
		}
		return std::numeric_limits<double>::max();
	}

	double heuristic_map_t::get_cost(const double x, const double y)
	{
		double global[2] = {x, y};
		int g_map[2];
		global_to_global_map_space(g_map, global);
		if (g_map[0] < grid_length && g_map[1] < grid_width && g_map[0] >= 0 && g_map[1] >=0)
		{
			if (global_cost_map[g_map[0]][g_map[1]] == -1) return 1;
			// if (global_cost_map[g_map[0]][g_map[1]] == 1)
			// 	return std::numeric_limits<double>::max();
			return global_cost_map[g_map[0]][g_map[1]];
		}
		return 1;
	}
	double heuristic_map_t::get_cost(const int x, const int y)
	{
		if (x < grid_length && y < grid_width && x >= 0 && y >=0)
		{
			if (global_cost_map[x][y] == -1) return 1;
			// if (global_cost_map[g_map[0]][g_map[1]] == 1)
			// 	return std::numeric_limits<double>::max();
			return global_cost_map[x][y];
		}
		return 1;
	}

	double heuristic_map_t::get_score(const int x, const int y)
	{
		if (x < grid_length && y < grid_width && x >= 0 && y >=0)
		{
			if (global_cost_map[x][y] == -1) return std::numeric_limits<double>::max();
			return global_h_map[x][y];
		}
		return std::numeric_limits<double>::max();

	}

	heuristic_map_t::~heuristic_map_t()
	{
		free_map(global_cost_map, grid_length);
		free_map(global_h_map, grid_length);
		free_map(seen_map, grid_length);
	}

	void heuristic_map_t::free_map(double ** g, int grid_h)
	{
		for(int i = 0; i < grid_h; i++)
		{
			free(g[i]);
		}
		free(g);
	}

	void heuristic_map_t::free_map(bool ** g, int grid_h)
	{
		for(int i = 0; i < grid_h; i++)
		{
			free(g[i]);
		}
		free(g);
	}

	void heuristic_map_t::replan(const double world_origin[2], int dim[2], double start[2], double p_density)
	{
		double dx = world_origin[0] - dim[0] / 2.0;
		double dy = world_origin[1] - dim[1] / 2.0;
		double ** local_cost_map = init_rand_map(start, dim, dx, dy, p_density=p_density);
		add_to_global_map(local_cost_map, dim, dx, dy);
		dijkstra();
	}

	double ** heuristic_map_t::init_rand_map(double _start[2], int dim[2], double dx, double dy, double p_density)
	{
		int start[2];
		global_to_local_map_space(start, _start, dim, dx, dy);
		int startx = start[0], starty = start[1];
		double ** cost_map = (double **) malloc(dim[0] * sizeof(double *));
		for(int i = 0; i < dim[0]; i++)
		{
			cost_map[i] = (double *) malloc(dim[1] * sizeof(double));
			for(int j = 0; j < dim[1]; j++)
			{
				int local[2] = {i, j};
				int global_map[2];
				local_map_to_global_map_space(global_map, local, dim, dx, dy);
				if (seen_map[global_map[0]][global_map[1]]) cost_map[i][j] = global_cost_map[global_map[0]][global_map[1]];
				else
				{
					cost_map[i][j] = global_cost_map[global_map[0]][global_map[1]];	// easy terrain
					// cost_map[i][j] = 0;	// easy terrain

				}
			}
		}

		std::srand(12345);
		std::default_random_engine generator;
		std::uniform_real_distribution<double> distribution1(0.1 * std::min(dim[1], dim[0]), std::min(dim[1], dim[0]));
		std::uniform_real_distribution<double> distribution2(0.0, 0.05 * std::min(dim[1], dim[0]));
		double mu = 0;
		// double obstacle_frequency = 0.2*cell_size;	// 1/100 cells is an obstacle
		double obstacle_frequency = p_density*cell_size;	// 1/100 cells is an obstacle
		for(int i = 0; i < dim[0]; i++)
		{
			for(int j = 0; j < dim[1]; j++)
			{
				int local[2] = {i, j};
				int global_map[2];
				local_map_to_global_map_space(global_map, local, dim, dx, dy);
				if (seen_map[global_map[0]][global_map[1]])
				{
					continue;
				}
				if ((std::abs(i - startx) < 1/cell_size && std::abs(j - starty) < 1/cell_size) || (std::abs(j - goaly) < 1/cell_size && std::abs(i - goalx) < 1/cell_size) )
					continue;
				if (((double) std::rand() / (RAND_MAX)) < obstacle_frequency)
				{

					//cost_map[i][j] =  (((double) std::rand() / (RAND_MAX)) * 0.5) + 0.6;
					//cost_map[i][j] = std::min(cost_map[i][j], 1.0);
					cost_map[i][j] = 1.0;
					int obs_area_length; // = std::rand() % std::min(dim[1], dim[0]);		// area affected by terrain near obstacle
					if (cost_map[i][j] == 1)
					{
						obs_area_length = distribution1(generator);
					}
					else
						obs_area_length = distribution2(generator);
					//std::cout << i << " " << j << " " << cost_map[i][j] << " " << obs_area_length << std::endl;
					//int obs_area_length = std::rand() % dim[0];
					//int obs_area_width = std::rand() % dim[1];
					int obs_area_width = obs_area_length;
					// Fill buckets
					int x_min = std::max(0, i - obs_area_length/2), x_max = std::min(dim[0], i + obs_area_length/2);
					for (int x = x_min; x < x_max; x++)
					{
						int y_min = std::max(0, j - obs_area_width/2), y_max = std::min(dim[1], j + obs_area_width/2);
						for (int y = y_min; y < y_max; y++)
						{
							if (x == i && y == j)
								continue;
							double sigma = 2/cell_size;
							double d = sqrt((i-x)*(i-x) + (j-y)*(j-y));
							cost_map[x][y] = std::max(cost_map[x][y], cost_map[i][j] * exp( -((d-mu)*(d-mu) / (2.0 * sigma*sigma)) ) );
						}
					}
				}
			}
		}
		// // Free start area
		for(int i = std::max(0, (int) (startx - 1.0/cell_size)); i < std::min(dim[0], (int) (startx + 1.0/cell_size)); i++)
		{
			for(int j = std::max(0, (int) (starty - 1.0/cell_size)); j < std::min(dim[1], (int) (starty + 1.0/cell_size)); j++)
			{
				int local[2] = {i, j};
				int global_map[2];
				local_map_to_global_map_space(global_map, local, dim, dx, dy);
				if (seen_map[global_map[0]][global_map[1]])
					continue;
				cost_map[i][j] = 0;
			}
		}
		// // Free goal area
		// for(int i = std::max(0, (int) (local_goal[0] - 1.0/cell_size)); i < std::min(dim[0], (int) (local_goal[0] + 1.0/cell_size)); i++)
		// {
		// 	for(int j = std::max(0, (int) (local_goal[1] - 1.0/cell_size)); j < std::min(dim[1], (int) (local_goal[1] + 1.0/cell_size)); j++)
		// 	{
		// 		int local[2] = {i, j};
		// 		int global_map[2];
		// 		local_map_to_global_map_space(global_map, local, dim, dx, dy);
		// 		if (seen_map[global_map[0]][global_map[1]])
		// 			continue;
		// 		cost_map[i][j] = 0;
		// 	}
		// }
		return cost_map;

	}

	void heuristic_map_t::add_to_global_map(double ** local_cost_map, int dim[2], double dx, int dy)
	{
		for (int i = 0; i < dim[0]; i++)
		{
			for (int j = 0; j < dim[1]; j++)
			{
				int local[2] = {i, j};
				int gm[2];	// global map coords
				local_map_to_global_map_space(gm, local, dim, dx, dy);
				global_cost_map[gm[0]][gm[1]] = local_cost_map[i][j];
				seen_map[gm[0]][gm[1]] = true;
			}
		}
	}

	void heuristic_map_t::global_to_local_map_space(int out[2], double global[2], int dim[2], double dx, double dy)
	{
		// x
		out[0] = global[0] / cell_size - dx;
		out[0] = std::min(out[0], dim[0]-1);
		out[0] = std::max(out[0], 0);
		// y
		out[1] = global[1] / cell_size - dy;
		out[1] = std::min(out[1], dim[1]-1);
		out[1] = std::max(out[1], 0);
	}

	void heuristic_map_t::local_map_to_global_space(double out[2], int local[2], int dim[2], double dx, double dy)
	{
		// x
		out[0] = (local[0] + dx)* cell_size;
		// y
		out[1] = (local[1] + dy)* cell_size;
	}

	double heuristic_map_t::cell_round(double a)
	{
		double lower = std::floor(a);
		while (lower < a-cell_size)
			lower += cell_size;
		double upper = (lower+cell_size-a)*(lower+cell_size-a);
		double l_cmp = (lower-a)*(lower-a);
		double out = l_cmp < upper ? lower : lower+cell_size;
		return out;
	}

	bool heuristic_map_t::is_collision_free(const space_point_t point)
	{
		double global[2] = {point->at(0),point->at(1)};
		int g_map[2];
		global_to_global_map_space(g_map, global);
		// std::cout<< global_cost_map[g_map[0]][g_map[1]]<< "\t"<<in_grid(g_map[0], g_map[1])<<"\t"<<(global_cost_map[g_map[0]][g_map[1]]!=PRX_UNSEEN_CELL)<<std::endl;
		return in_grid(g_map[0], g_map[1])&&global_cost_map[g_map[0]][g_map[1]]!=1 && global_cost_map[g_map[0]][g_map[1]]!=PRX_UNSEEN_CELL;
	}

	std::vector<double> heuristic_map_t::get_local_map(double _x, double _y, int size)
	{
		double global[2] = {_x,_y};
		int map_coords[2];
		global_to_global_map_space(map_coords, global);
		int x = map_coords[0];
		int y = map_coords[1];
		std::vector<double> out; // of size*size length
		for (int i = x-((int)(size/2)); i <= x+((int)(size/2)); i++)
		{
			for (int j = y-((int)(size/2)); j <= y+((int)(size/2)); j++)
			{
				prx_assert((i < grid_length && i >=0 && j < grid_width && j >= 0), "Error in local map boundaries: " +std::to_string(i)+" "+std::to_string(j) + " "+std::to_string(x)+" "+std::to_string(y));
				if (i < grid_length && i >=0 && j < grid_width && j >= 0)
				{
					out.push_back(global_h_map[i][j]);
				} else {
					// out.push_back(10000000000000);
					out.push_back(std::numeric_limits<double>::max());
				}

			}
		}
		return out;
	}
	std::vector<int> heuristic_map_t::get_local_obs_map(double _x, double _y, int size)
	{
		double global[2] = {_x,_y};
		int map_coords[2];
		global_to_global_map_space(map_coords, global);
		int x = map_coords[0];
		int y = map_coords[1];
		std::vector<int> out; // of size*size length
		for (int i = x-((int)(size/2)); i <= x+((int)(size/2)); i++)
		{
			for (int j = y-((int)(size/2)); j <= y+((int)(size/2)); j++)
			{
				int out_val = 1;
				if (i < grid_length && i >=0 && j < grid_width && j >= 0)
				{
					if (global_cost_map[i][j] != 1) out_val = 0;
				}
				out.push_back(out_val);

			}
		}
		return out;
	}

	void heuristic_map_t::global_to_global_map_space(int out[2], double global[2])
	{
		// x
		double gx = global[0];
		out[0] = gx / cell_size - delta_x;
		out[0] = std::min(out[0], grid_length-1);
		out[0] = std::max(out[0], 0);
		// y
		double gy = global[1];
		out[1] = gy / cell_size - delta_y;
		out[1] = std::min(out[1], grid_width-1);
		out[1] = std::max(out[1], 0);
	}

	void heuristic_map_t::global_map_to_global_space(double out[2], int global_map[2])
	{
		// x
		out[0] = (global_map[0] + delta_x)* cell_size;// - cell_size/2.0;
		// y
		out[1] = (global_map[1] + delta_y)* cell_size;// - cell_size/2.0;
	}

	void heuristic_map_t::local_map_to_global_map_space(int out[2], int local[2], int dim[2], double dx, double dy)
	{
		double global_coords[2];
		local_map_to_global_space(global_coords, local, dim, dx, dy);
		global_to_global_map_space(out, global_coords);
	}

	double heuristic_map_t::add_time_multiplier(double cost, double x, double y)
	{
		int coord[2];
		double global[2] = {x, y};
		global_to_global_map_space(coord, global);
		double traversability = global_cost_map[coord[0]][coord[1]];
		prx_assert(!((traversability > 1 || traversability < 0) && traversability != PRX_UNSEEN_CELL), "Invalid traversability value found");
		// if (traversability > 1 || traversability < 0) return cost;
		return cost * (exp(4.6*traversability));
	}

	void heuristic_map_t::dijkstra()
	{
		// std::cout<<"Starting Dijkstra"<<std::endl;
		// double ** g = create_cost_to_go_map();
		if (global_h_map==NULL)
		{
			global_h_map = create_cost_to_go_map();
		}
		else
		{
			create_cost_to_go_map(global_h_map);
		}
		std::vector<node_t> fringe;
		if(!in_grid(goalx, goaly)) get_k_nearest_neighbors(fringe);
		else fringe = get_neighbors(goalx, goaly);
		std::make_heap(fringe.begin(), fringe.end());
		std::vector<std::vector<seen_node_t>> seen;
		for (int i = 0; i < grid_length; i++)
		{
			std::vector<seen_node_t> seen1;
			for (int j = 0; j < grid_width; j++)
			{
				if (i==goalx && j==goaly)
				{
					seen_node_t new_n = seen_node_t(&fringe.back());
					seen1.push_back(new_n);
				}
				else
				{
					seen_node_t new_n = seen_node_t(nullptr);
					seen1.push_back(new_n);
				}
			}
			seen.push_back(seen1);
		}
		// std::priority_queue<node_t> obs_fringe;
		std::vector<node_t> obs_fringe;
		// std::clock_t h_map_time = std::clock();
		while(!fringe.empty())
		{
			// print_h_map();
			node_t v_cur = fringe.front();
			std::pop_heap(fringe.begin(),fringe.end()); fringe.pop_back();
			if (seen[v_cur.x][v_cur.y].n != nullptr) continue;
			if (use_min_cost) global_h_map[v_cur.x][v_cur.y] = v_cur.g;

			seen[v_cur.x][v_cur.y].n = &v_cur;
			std::vector<node_t> neighbors = get_neighbors(v_cur.x, v_cur.y, global_h_map);
			for(node_t n : neighbors)
			{
				// if (seen[n.x][n.y].n != nullptr || global_cost_map[n.x][n.y] == -1) continue;
				if (global_cost_map[n.x][n.y] == -1) continue;
				if (global_cost_map[n.x][n.y] == 1.0)
				{
					// std::cout << n.x<<"\t"<<n.y<<"\t"<<global_h_map[n.x][n.y]<<std::endl;
					if (use_obs_h)
					{
						double g_cur = 1 + global_h_map[v_cur.x][v_cur.y] + get_num_obs_neighbors(v_cur.x, v_cur.y);
						prx_assert((global_h_map[v_cur.x][v_cur.y] != std::numeric_limits<double>::max()), "ugh");
						if (g_cur >= global_h_map[n.x][n.y]) continue;
						seen[n.x][n.y].n = nullptr;
						n.g = g_cur;
						global_h_map[n.x][n.y] = g_cur;
						obs_fringe.push_back(n);
					}
					else global_h_map[n.x][n.y] = std::numeric_limits<double>::max();
					// seen[n.x][n.y].n = &n;
					continue;
				}
				double g_cur;
				// double cur_cost = 0.01*cell_size/1.0*exp(4.6*global_cost_map[n.x][n.y]); // divide by 1.0 because max velocity
				double cur_cost = 1; // divide by 1.0 because max velocity
				// if (use_learning) cur_cost = .99*cell_size/1.0;
				
				// warning: expression result unused [-Wunused-value]
				// if (global_cost_map[n.x][n.y] == PRX_UNSEEN_CELL) 0.01*cell_size/1.0; // optimistic treatment of unseen areas
				g_cur = cur_cost + v_cur.g + get_num_obs_neighbors(v_cur.x, v_cur.y);
				if (g_cur >= global_h_map[n.x][n.y]) continue;
				n.g = g_cur;
				global_h_map[n.x][n.y] = g_cur;
				fringe.push_back(n);
				std::push_heap(fringe.begin(), fringe.end());
			}
		}
		// double _t = (std::clock() - h_map_time ) / (double) CLOCKS_PER_SEC;
                // std::cout<<"TIME:\t"<< _t <<std::endl;
		if (use_obs_h)
		{
			std::make_heap(obs_fringe.begin(), obs_fringe.end());
			/*
			std::vector<node_t> neighbors = get_neighbors(goalx, goaly);
			for (auto n : neighbors)
			{
				obs_fringe.push(n);
			}
			for (int i = 0; i < grid_length; i++)
			{
				for (int j = 0; j < grid_width; j++)
				{
					if (i==goalx && j==goaly)
					{
						node_t goal_node = obs_fringe.top();
						seen_node_t new_n = seen_node_t(&goal_node);
						seen[i][j] = new_n;
					}
					else
					{
						seen_node_t new_n = seen_node_t(nullptr);
						seen[i][j] = new_n;
					}
				}
			}
			*/
			while(!obs_fringe.empty())
			{
				node_t v_cur = obs_fringe.front();
				std::pop_heap(obs_fringe.begin(),obs_fringe.end()); obs_fringe.pop_back();
				// node_t v_cur = obs_fringe.top();
				// obs_fringe.pop();
				if (seen[v_cur.x][v_cur.y].n != nullptr) continue;
				seen[v_cur.x][v_cur.y].n = &v_cur;
				// if (v_cur.g > global_h_map[v_cur.x][v_cur.y]) continue;

				std::vector<node_t> neighbors = get_neighbors(v_cur.x, v_cur.y, global_h_map);
				for(node_t n : neighbors)
				{
					// if (seen[n.x][n.y] != nullptr)
					// 	continue;
					if (global_cost_map[n.x][n.y] == 1.0 || global_h_map[n.x][n.y] == std::numeric_limits<double>::max())
					{
						//
						// double g_cur = 0.01*cell_size/1.0*exp(4.6*global_cost_map[n.x][n.y]) + global_h_map[v_cur.x][v_cur.y] + get_num_obs_neighbors(v_cur.x, v_cur.y);
						double g_cur = 1 + global_h_map[v_cur.x][v_cur.y] + get_num_obs_neighbors(v_cur.x, v_cur.y);
						prx_assert((global_h_map[v_cur.x][v_cur.y] != std::numeric_limits<double>::max()), "ugh");
						if (g_cur >= global_h_map[n.x][n.y]) continue;
						seen[n.x][n.y].n = nullptr;
						n.g = g_cur;
						global_h_map[n.x][n.y] = g_cur;
					}
					if (seen[n.x][n.y].n == nullptr) obs_fringe.push_back(n);
					std::push_heap(obs_fringe.begin(), obs_fringe.end());
				}
			}
		}
	}

	double ** heuristic_map_t::create_cost_to_go_map()
	{
		double ** g = (double **) malloc(grid_length * sizeof(double *));
		for(int i = 0; i < grid_length; i++)
		{
			g[i] = (double *) malloc(grid_width * sizeof(double));
			for(int j = 0; j < grid_width; j++)
			{
				if (goalx == i && goaly == j)
					g[i][j] = 0;
				else
					g[i][j] = std::numeric_limits<double>::max();
			}
		}
		return g;
	}

	void heuristic_map_t::create_cost_to_go_map(double ** g)
	{
		for(int i = 0; i < grid_length; i++)
		{
			for(int j = 0; j < grid_width; j++)
			{
				if (goalx == i && goaly == j)
					g[i][j] = 0;
				else
					g[i][j] = std::numeric_limits<double>::max();
			}
		}
	}

	std::vector<node_t> heuristic_map_t::get_neighbors(int x, int y)
	{
		int y_max = grid_length - 1;
		int x_max = grid_width  - 1;
		std::vector<node_t> neighbors;
		if (x < 0 || x > x_max || y < 0 || y > y_max)
		{
			return neighbors;
		}

		for(int i = std::fmax(0, x-1); i <= std::fmin(x+1, x_max); i++)
		{
			for(int j = std::fmax(0, y-1); j <= std::fmin(y+1, y_max); j++)
			{
				if (i == x && j == y) continue;
				if (use_learning&&use_obs_h)
					neighbors.push_back(node_t(i, j, .99*cell_size/1.0));
				else
				{
					// neighbors.push_back(node_t(i, j, exp(4.6*global_cost_map[i][j])));
					neighbors.push_back(node_t(i, j, 1));

				}
			}

		}

		return neighbors;

	}

	std::vector<node_t> heuristic_map_t::get_neighbors(int x, int y, double ** g)
	{
		int y_max = grid_length - 1;
		int x_max = grid_width  - 1;
		std::vector<node_t> neighbors;
		if (x < 0 || x > x_max || y < 0 || y > y_max)
		{
			return neighbors;
		}

		for(int i = std::fmax(0, x-1); i <= std::fmin(x+1, x_max); i++)
		{
			for(int j = std::fmax(0, y-1); j <= std::fmin(y+1, y_max); j++)
			{
				if (i == x && j == y) continue;
				neighbors.push_back(node_t(i, j, g[i][j]));
			}

		}

		return neighbors;

	}

	int heuristic_map_t::get_num_obs_neighbors(int x, int y)
	{
		int y_max = grid_length - 1;
		int x_max = grid_width  - 1;
		int neighbors = 0;
		if (x < 0 || x > x_max || y < 0 || y > y_max)
		{
			return neighbors;
		}

		for(int i = std::fmax(0, x-1); i <= std::fmin(x+1, x_max); i++)
		{
			for(int j = std::fmax(0, y-1); j <= std::fmin(y+1, y_max); j++)
			{
				if (i == x && j == y) continue;
				if (global_cost_map[i][j] == 1) neighbors++;
			}

		}

		return neighbors;

	}

	// unused, deprecated
	/*
	double heuristic_map_t::sum_parent_cost(int x, int y, double ** g, int *** p)
	{
		double global[2] = {goalx, goaly};	// true global coords
		int gm[2];	// global map
		global_to_global_map_space(gm, global);
		int sum = 0;
		while ((x != gm[0] && y != gm[1]) && (x != -1 && y != -1))
		{
			if(x==-1 || y==-1)
				printf("error\n");
			int par_x = p[x][y][0];
			int par_y = p[x][y][1];
			sum += g[x][y];
			x = par_x;
			y = par_y;
		}
		return sum;
	}
	*/

	bool heuristic_map_t::in_grid(int x, int y)
	{
		// local space
		if(x >= 0 && x < grid_length && y >= 0 && y < grid_width) return true;
		else return false;
	}

	double heuristic_map_t::euclidean_distance(double x, double y)
	{
		double globals[2];
		int goal[2] = {goalx, goaly};
		global_map_to_global_space(globals, goal);
		double globalx = globals[0];
		double globaly = globals[1];
		return sqrt((globalx - x)*(globalx - x) + (globaly - y)*(globaly - y));
	}

	std::vector<node_t> heuristic_map_t::insertion_sort(std::vector<node_t> neighbors, int x, int y)
	{
		int global_map_coords[2] = {x,y};
		double globals[2];
		global_map_to_global_space(globals, global_map_coords);
		double cost = add_time_multiplier(euclidean_distance(x, y), globals[0], globals[1]);
		if (neighbors.size() == 0)
		{
			neighbors.push_back(node_t(x, y, cost));
		}
		else
		{
			for(int i = 0; i < neighbors.size(); i++)
			{
				if (cost < neighbors[i].g)
				{
					neighbors.insert(neighbors.begin()+i, node_t(x, y, cost));
					return neighbors;
				}
			}
		}
		return neighbors;
	}

	void heuristic_map_t::get_k_nearest_neighbors(std::vector<node_t> out)
	{
		std::vector<node_t> neighbors;
		for(int x = 0, y = 0; x < grid_length; x++)
		{
			neighbors = insertion_sort(neighbors, x, y);
		}
		for(int x = 0, y = 1; y < grid_width; y++)
		{
			neighbors = insertion_sort(neighbors, x, y);
		}
		for(int x = 0, y = grid_width-1; x < grid_length; x++)
		{
			neighbors = insertion_sort(neighbors, x, y);
		}
		for(int x = grid_length-2, y = 0; y < grid_width; y++)
		{
			neighbors = insertion_sort(neighbors, x, y);
		}
		for (int i = 0; i < knn; i++) out.push_back(neighbors[i]);
	}

	void heuristic_map_t::print_cost_map()
	{
		for (int i = grid_length-1; i >=0 ; i--)
		{
			for (int j = 0; j < grid_width; j++)
			{
				std::cout<<global_cost_map[i][j]<<"\t";
			}
			std::cout<<std::endl;
		}
	}

	void heuristic_map_t::print_h_map()
	{
		for (int i = grid_length-1; i >=0 ; i--)
		{
			for (int j = 0; j < grid_width; j++)
			{
				if (global_h_map[i][j] == std::numeric_limits<double>::max()) std::cout<<-1<<" ";
				else std::cout<<global_h_map[i][j]<<" ";
			}
			std::cout<<std::endl;
		}
		std::cout<<std::endl;
	}

	std::vector<std::vector<bool>> extract_map(const std::string map_fname)
	{
		std::vector<std::vector<bool>> extracted_map;
		std::ifstream ifs(map_fname);
		std::string line;
		int height, width;

		std::getline(ifs,line); // Assuming map type is octile, so skip

		std::getline(ifs,line);
		int i = 0;
		for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
		line = line.substr(i,line.length() - i);
		height = std::atoi(line.c_str());
		std::cout << "Height: " << height;

		std::getline(ifs,line);
		for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
		line = line.substr(i,line.length() - i);
		width = std::atoi(line.c_str());
		std::cout << " Width: " << width << std::endl;

		std::getline(ifs,line); // This is the line that says "map"

		for (i = 0; i < height; i++)
		{
			std::vector<bool> map_line;
			std::getline(ifs,line);
			for (int j = 0; j < width; j++)
			{
				//if (line[j] == '.') map_line.push_back(true);
				//else map_line.push_back(false);
				map_line.push_back(line[j] == '.');
			}
			extracted_map.push_back(map_line);
		}

		return extracted_map;
	}
	// Constructor for CoRL
	heuristic_map_t::heuristic_map_t(const double world_origin_x,
					const double world_origin_y,
					const double _goalx,
					const double _goaly,
					const int grid_h,
					const int grid_w,
					const double c_size,
					const std::string map_fname,
					const int padding
	)
	{
		grid_length = grid_h + 2 * padding;
		grid_width = grid_w + 2 * padding;
		cell_size = c_size;
		use_min_cost = true;
		use_learning = false;
		use_obs_h=true;
		delta_x = world_origin_x - grid_length / 2.0;
		delta_y = world_origin_y - grid_width / 2.0;
		double global_goal[2] = {_goalx, _goaly};
		int goal_coords[2];
		global_to_global_map_space(goal_coords, global_goal);
		goalx = goal_coords[0];
		goaly = goal_coords[1];
		global_cost_map = (double **) malloc(grid_length * sizeof(double *));
		global_h_map = (double **) malloc(grid_length * sizeof(double *));
		seen_map = (bool **) malloc(grid_length * sizeof(bool *));

		std::vector<std::vector<bool>> extracted_map = extract_map(map_fname);

		for(int i = 0; i < grid_length; i++)
		{
			global_cost_map[i] = (double *) malloc(grid_width * sizeof(double));
			global_h_map[i] = (double *) malloc(grid_width * sizeof(double));
			seen_map[i] = (bool *) malloc(grid_width * sizeof(bool));
			for(int j = 0; j < grid_width; j++)
			{
				// std::cout <<i<<"\t"<<j<<std::endl;
				if (i < padding || j < padding || i >= grid_length-padding || j >= grid_width-padding) global_cost_map[i][j] = 1;
				else if (extracted_map[(j-padding)][(i-padding)]) global_cost_map[i][j] = 0;
		                else global_cost_map[i][j] = 1;
				// else if (line[j-padding] == '.') global_cost_map[i][j] = 0;
		                // else global_cost_map[i][j] = 1;
				seen_map[i][j] = false;
				global_h_map[i][j] = 0;
			}
		}
		double max_val=1.;
	        // for(int x=0;x<grid_length;x++)
	        // {
	        //         for(int y=0;y<grid_width;y++)
	        //         {
	        //                 // std::cout << get_score(x,y) << "\t";
	        //                 max_val = max_val > get_score(x,y) || get_score(x,y) == std::numeric_limits<double>::max() ? max_val : get_score(x,y);
	        //                 // max_val = max_val > global_cost_map[x][y] || global_cost_map[x][y] == std::numeric_limits<double>::max() ? max_val : global_cost_map[x][y];
	        //         }
	        //         // std::cout << std::endl;
	        // }
	        std::cout << max_val << std::endl;
		std::ofstream outfile;
	        std::string map_string = "P2\n"+std::to_string(grid_length)+" "+std::to_string(grid_width)+"\n"+std::to_string((int)max_val)+"\n";
	        // for (int j = grid_width-1; j >= 0; j--)
	        for (int j = 0; j < grid_width; j++)
	        {
			// for(int j = 0; j < grid_width; j++)
			for (int i = 0; i < grid_length; i++)
	                {
	                        // double d_val = global_h_map[i][j];
	                        double d_val = global_cost_map[i][j];
	                        int val = d_val == std::numeric_limits<double>::max() ? 0 : (int)max_val-d_val;
	                        map_string += std::to_string(val);
	                        if (i != grid_length-1) map_string += " ";
	                }
	                map_string += "\n";
	        }
	        outfile.open("/home/milkkarten/test1.pgm");
	        outfile << map_string;
	        outfile.close();
	        std::cout<<"Obstacle map successfully saved at /home/milkkarten/test1.pgm"<<std::endl;

		dijkstra();
	}

	// Adaptation of the Bresenham Line-Drawing Algorithm
	bool heuristic_map_t::line_of_sight(int x0, int y0, int x1, int y1)
	{
		int f = 0;
		int dy = y1 - y0;
		int dx = x1 - x0;
		int sy = 1;
		if (dy < 0)
		{
			dy = -dy;
			sy = -1;
		}
		int sx = 1;
		if (dx < 0)
		{
			dx = -dx;
			sx = -1;
		}
		if (dx >= dy)
		{
			while (x0 != x1)
			{
				f += dy;
				if (f >= dx)
				{
					if (global_cost_map[x0+(sx-1)/2][y0+(sy-1)/2]  == 1) return false;
					y0 = y0 + sy;
					f = f - dx;
				}
				if (f != 0 && global_cost_map[x0+(sx-1)/2][y0+(sy-1)/2] == 1) return false;
				if (dy == 0 && global_cost_map[x0+(sx-1)/2][y0-1] == 1) return false;
				x0 += sx;
			}
		}
		else
		{
			while (y0 != y1)
			{
				f += dx;
				if (f >= dy)
				{
					if (global_cost_map[x0+(sx-1)/2][y0+(sy-1)/2]  == 1) return false;
					x0 = x0 + sx;
					f = f - dy;
				}
				if (f != 0 && global_cost_map[x0+(sx-1)/2][y0+(sy-1)/2] == 1) return false;
				if (dx == 0 && get_cost(x0, y0+(sy-1)/2) == 1 && get_cost(x0-1,y0+(sy-1)/2) == 1) return false;
				// if (dx == 0 && global_cost_map[x0][y0+(sy-1)/2] == 1 && global_cost_map[x0-1][y0+(sy-1)/2] == 1) return false;
				y0 += sy;
			}
		}
		return true;
	}

	std::vector<std::vector<std::vector<int>>> heuristic_map_t::compute_vector_field()
	{
		double size = grid_length;
		std::clock_t time = std::clock();
		// Allocate vector map
		std::vector<std::vector<std::vector<int>>> vector_map;
		for (int i = 0; i < grid_length; i++)
		{
			std::vector<std::vector<int>> vector_row;
			for (int j = 0; j < grid_width; j++)
			{
				std::vector<int> vector;
				vector.push_back(i);
				vector.push_back(j);
				vector_row.push_back(vector);
			}
			vector_map.push_back(vector_row);
		}
		// Compute vector field of whole map
		std::vector<std::vector<seen_node_t>> seen;
		for (int i = 0; i < grid_length; i++)
		{
			std::vector<seen_node_t> seen1;
			for (int j = 0; j < grid_width; j++)
			{
				seen_node_t new_n = seen_node_t(nullptr);
				seen1.push_back(new_n);
			}
			seen.push_back(seen1);
		}
		for (int x = 0; x < size; x++)
		{
			for (int y = 0; y < size; y++)
			{
				if (get_cost(x,y)==1) continue;
				double min_val = global_h_map[x][y];
				// BFS
				std::vector<node_t> fringe_init = get_neighbors(x, y);
				std::queue<node_t> fringe;
				for (auto node : fringe_init) fringe.push(node);
				for (int i = 0; i < size; i++)
				{
					for (int j = 0; j < size; j++)
					{
						if (i==x && j==y) seen[i][j].n = &fringe.front();
						else seen[i][j].n = nullptr;
					}
				}
				while(!fringe.empty())
				{
					node_t v_cur = fringe.front();
					fringe.pop();
					// if (seen[v_cur.x][v_cur.y].n != nullptr) continue;
					// Check if min value
					if (global_h_map[v_cur.x][v_cur.y] < min_val)
					{
						min_val = global_h_map[v_cur.x][v_cur.y];
						vector_map[x][y][0] = v_cur.x;
						vector_map[x][y][1] = v_cur.y;
					}

					// seen[v_cur.x][v_cur.y].n = &v_cur;
					std::vector<node_t> neighbors = get_neighbors(v_cur.x, v_cur.y);
					for(node_t n : neighbors)
					{

						if (	global_cost_map[n.x][n.y] == -1 ||
							global_cost_map[n.x][n.y] == 1.0 ||
							seen[n.x][n.y].n != nullptr
							|| global_h_map[n.x][n.y] >= global_h_map[v_cur.x][v_cur.y]
						)
						{
							seen[n.x][n.y].n = &n;
							continue;
						}
						seen[n.x][n.y].n = &n;
						if (!line_of_sight(x, y, n.x, n.y)) continue;
						fringe.push(n);
					}
				}
			}
		}

		double _t = (std::clock() - time ) / (double) CLOCKS_PER_SEC;
		std::cout<<"VECTOR FIELD TIME:\t"<< _t <<std::endl;
		return vector_map;
	}

	// Constructor for CoRL
    heuristic_map_t::heuristic_map_t(const double world_origin_x,
                    const double world_origin_y,
                    const double _goalx,
                    const double _goaly,
                    const int grid_h,
                    const int grid_w,
                    const double c_size,
                    const std::vector<std::vector<bool>> extracted_map
    )
    {

        //std::vector<std::vector<bool>> extracted_map;
        /**    std::ifstream ifs(map_fname);
            std::string line;

            std::getline(ifs,line); // Assuming map type is octile, so skip

            std::getline(ifs,line);
            int i = 0;
            for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
            line = line.substr(i,line.length() - i);
            grid_length = std::atoi(line.c_str());
            std::cout << "Height: " << height;

            std::getline(ifs,line);
            for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
            line = line.substr(i,line.length() - i);
            grid_width = std::atoi(line.c_str());
            std::cout << " Width: " << width << std::endl;
            std::getline(ifs,line); // This is the line that says "map"
    **/
        prx_assert(extracted_map.size() > 0, "Empty extracted_map!");
        grid_length = extracted_map.size();
        grid_width = extracted_map[0].size();
        cell_size = c_size;
        use_min_cost = true;
        use_learning = false;
        use_obs_h=false;
        delta_x = world_origin_x - grid_length / 2.0;
        delta_y = world_origin_y - grid_width / 2.0;
        double global_goal[2] = {_goalx, _goaly};
        int goal_coords[2];
        global_to_global_map_space(goal_coords, global_goal);
        goalx = goal_coords[0];
        goaly = goal_coords[1];
        std::cout << "Goal: " << goalx << " " << goaly << std::endl;
        global_cost_map = (double **) malloc(grid_length * sizeof(double *));
        global_h_map = (double **) malloc(grid_length * sizeof(double *));
        seen_map = (bool **) malloc(grid_length * sizeof(bool *));

        for(int i = 0; i < grid_length; i++)
        {
            global_cost_map[i] = (double *) malloc(grid_width * sizeof(double));
            global_h_map[i] = (double *) malloc(grid_width * sizeof(double));
            seen_map[i] = (bool *) malloc(grid_width * sizeof(bool));
        }
        for(int i = 0; i < grid_length; i++)
        {
            for(int j = 0; j < grid_width; j++)
            //for(int j = grid_width-1; j >= 0; --j)
            {
                //if (line[j] == '.') global_cost_map[i][j] = 0;
                //        else global_cost_map[i][j] = 1;
                //global_cost_map[i][j] = !extracted_map[i][j];
                global_cost_map[i][j] = extracted_map[i][j]?0:1;
                seen_map[i][j] = false;
                global_h_map[i][j] = 0;
            }
        }

        dijkstra();

    }
}
