#pragma once

#include "prx/simulation/plant.hpp"
#include "prx/simulation/playback/trajectory.hpp"
#include "prx/utilities/geometry/geometry.hpp"

#include <vector>

namespace prx
{

	enum class info_geometry_t
	{
		LINE = 0,
		QUAD = 1,
		FULL_LINE = 2,
		CIRCLE = 3
	};

	class three_js_group_t
	{
	public:
		three_js_group_t(const std::vector<system_ptr_t>& in_plants,const std::vector<std::shared_ptr<movable_object_t>>& in_obstacles={});
		~three_js_group_t();

		void add_vis_infos(info_geometry_t info_type, std::vector<trajectory_t>& tree_vis, std::string body_name, space_t* state_space, std::string color="0x000000");

		void add_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name, space_t* state_space, std::string color="0x000000");

		void add_vis_infos(info_geometry_t info_type, std::vector<vector_t> positions, std::string color="0x000000", double radius=1 );

		// This fn generates more detailed visualization of the trajectories 
		 
		/**
		 * Generates detailed visualization of a trajectory. Time and memory expensive, should only be used for a small number of trajectories, i.e. solution(s).
		 * @param info_type   One of info_geometry_t::LINE or info_geometry_t::FULL_LINE. The latter generates more visually accurate trajectories at the cost of time-memory
		 * @param traj        Trajectory to generate the visualizations for
		 * @param body_name   Body to generate the trajectory visualization for
		 * @param state_space State space of the plant for which body_name is part of
		 * @param color       Color to use for the visualized trajectory.
		 */
		void add_detailed_vis_infos(info_geometry_t info_type, const trajectory_t& traj, std::string body_name, space_t* state_space, std::string color="0xFF0000");

		/**
		 * Creates an animation for the trajectory using the state space. If the trajectory is empty, only puts the system at start_state.
		 * @param traj        Trajectory to animate
		 * @param state_space The space_t to use
		 * @param default_state Where to put the system if the trajectory is empty.
		 */
		void add_animation(const trajectory_t& traj, space_t* state_space, space_point_t default_state = nullptr);

		void update_vis_infos();

		void snapshot_state(double timestamp);

		void output_html(std::string filename);

		void add_tree_log(std::string log_name, space_t* state_space);


	protected:

		struct vis_info_t
		{
			vector_t position;
			quaternion_t rotation;
			std::weak_ptr<transform_t> transform;
			geometry_type_t geom_type;
			std::vector<double> geom_params;
			std::string body_name;
			std::string color;
			void update_info();
		};

		struct info_geom_params_t
		{
			info_geometry_t first;
			std::vector<vector_t> second;
			std::string third;
			double fourth;
		};

		// TODO: Change DIRTMP to whatever name we end up using
		const std::string html_header_1 = "<!DOCTYPE html><html><head><link rel=\"stylesheet\" href=\""+js_path+"style.css\"><meta charset=utf-8><title>DIRTMP Visualization</title>";
		const std::string html_header_2 = "<style>body { margin: 0; }canvas { width: 100%; height: 100%; display: block; }</style>";
		const std::string html_header_3 = "</head><body><button id=\"shot\">Screenshot</button><button id=\"bt_play\">Play</button><div class=\"slidecontainer\">";
		const std::string html_header_4 = "<input type=\"range\" min=\"0\" max=\"1000\" value=\"500\" class=\"slider\" id=\"time_slider\">";
		const std::string html_header_5 = "</div><script src=\""+js_path+"three.js\"></script><script src=\""+js_path+"map_controls.js\">";
		const std::string html_header_6 = "</script><script src=\""+js_path+"prx.js\"></script><script>";
		const std::string html_header = html_header_1 + html_header_2 + html_header_3 + html_header_4 + html_header_5 + html_header_6;
		// const std::string html_header = "<!DOCTYPE html><html><head><meta charset=utf-8><title>DIRTMP Visualization</title><style>body { margin: 0; }canvas { width: 100%; height: 100%; display: block; }</style></head><body><button id=\"shot\">Screenshot</button><div class=\"slidecontainer\"><input type=\"range\" min=\"1\" max=\"100\" value=\"50\" class=\"slider\" id=\"myRange\"></div><script src=\""+js_path+"three.js\"></script><script src=\""+js_path+"map_controls.js\"></script><script src=\""+js_path+"prx.js\"></script><script>";
		const std::string html_footer = "animate();</script></body></html>";
		// const std::string html_footer = "</script></body></html>";

		void update_plants();

		std::vector<info_geom_params_t> info_geoms;
		std::vector<system_ptr_t> plants;
		std::vector<std::shared_ptr<vis_info_t>> state_infos;
		std::vector<std::vector<double>> plant_animation_params;
		std::vector<std::shared_ptr<vis_info_t>> obstacle_infos;
		std::string tree_log_file;
		int state_space_dim;

	};
}
