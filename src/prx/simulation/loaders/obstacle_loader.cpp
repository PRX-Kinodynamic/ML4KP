#include "prx/simulation/loaders/obstacle_loader.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"
#include "prx/utilities/geometry/basic_geoms/cylinder.hpp"
#include "prx/utilities/geometry/basic_geoms/sphere.hpp"




namespace prx
{
// #ifndef BULLET_NOT_BUILT
//     std::pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>> load_obstacles(std::string obstacles_file, b3RobotSimulatorClientAPI* sim){
// #else
    std::pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>> load_obstacles(std::string obstacles_file){
// #endif
	
		if(obstacles_file=="")
		{
			std::cout<<"Trying to load an empty obstacle file. No obstacles loaded"<<std::endl;
			return std::make_pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>>({},{});
		}
		param_loader obstacle_loader(obstacles_file);
		auto geometries_list = obstacle_loader["environment"]["geometries"];

		std::vector<std::shared_ptr<movable_object_t>> obstacle_list;
		std::vector<std::string> obstacle_names;
		for (auto geom : geometries_list)
		{
			std::string name = geom["name"].as<std::string>();
			auto geom_transform = geom["config"];
			auto geom_position = geom_transform["position"].as<std::vector<double>>();
			auto geom_orientation = geom_transform["orientation"].as<std::vector<double>>();
			transform_t obstacle_pose;
			obstacle_pose.linear() = quaternion_t(geom_orientation[3],geom_orientation[0],geom_orientation[1],geom_orientation[2]).toRotationMatrix();
			obstacle_pose.translation() = vector_t(geom_position[0],geom_position[1],geom_position[2]);
			auto geom_params = geom["collision_geometry"];
			auto geom_type = geom_params["type"].as<std::string>();
			int shapeType=-1;

			if (geom_type == "box")
			{

				auto dims = geom_params["dims"].as<std::vector<double>>();
				obstacle_list.push_back(create_obstacle(new box_t(name,dims[0],dims[1],dims[2],obstacle_pose)));
				obstacle_names.push_back(name);
			}
			else if (geom_type == "cylinder")
			{
				double radius = geom_params["radius"].as<double>();
				double height = geom_params["height"].as<double>();
				obstacle_list.push_back(create_obstacle(new cylinder_t(name,radius,height,obstacle_pose)));
				obstacle_names.push_back(name);

			}
			else if (geom_type == "sphere")
			{
				double radius = geom_params["radius"].as<double>();
				obstacle_list.push_back(create_obstacle(new sphere_t(name,radius,obstacle_pose)));
				obstacle_names.push_back(name);
			}
			else
			{
				prx_throw("Obstacle loader can't load an obstacle of type: "<<geom_type);
			}

		}

		return std::make_pair(std::move(obstacle_names),std::move(obstacle_list));
	}
}
