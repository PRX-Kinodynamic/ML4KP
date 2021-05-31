#include "prx/bullet_sim/loaders/obstacle_loader.hpp"
#include "prx/utilities/geometry/basic_geoms/box.hpp"
#include "prx/utilities/geometry/basic_geoms/cylinder.hpp"
#include "prx/utilities/geometry/basic_geoms/sphere.hpp"



namespace prx
{
    std::pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>> load_obstacles(std::string obstacles_file, b3RobotSimulatorClientAPI* sim)
    {
 
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

			b3RobotSimulatorCreateVisualShapeArgs args_v;
			b3RobotSimulatorCreateCollisionShapeArgs args_c;			
			btVector3 basePosition;
			basePosition[0]=geom_position[0];
			basePosition[1]=geom_position[1];
			basePosition[2]=geom_position[2];
			btQuaternion baseOrientation(geom_orientation[3],geom_orientation[0],geom_orientation[1],geom_orientation[2]);

			if (geom_type == "box")
			{

				auto dims = geom_params["dims"].as<std::vector<double>>();
				obstacle_list.push_back(create_obstacle(new box_t(name,dims[0],dims[1],dims[2],obstacle_pose)));
				obstacle_names.push_back(name);
				shapeType=GEOM_BOX;	
				args_v.m_halfExtents[0]=dims[0]/2;
				args_v.m_halfExtents[1]=dims[1]/2;
				args_v.m_halfExtents[2]=dims[2]/2;
				args_c.m_halfExtents[0]=dims[0]/2;
				args_c.m_halfExtents[1]=dims[1]/2;
				args_c.m_halfExtents[2]=dims[2]/2;
			}
			else if (geom_type == "cylinder")
			{
				double radius = geom_params["radius"].as<double>();
				double height = geom_params["height"].as<double>();
				obstacle_list.push_back(create_obstacle(new cylinder_t(name,radius,height,obstacle_pose)));
				obstacle_names.push_back(name);
				shapeType=GEOM_CYLINDER;	
				args_v.m_radius=geom_params["radius"].as<double>();
				args_c.m_radius=geom_params["radius"].as<double>();
				args_v.m_height=geom_params["height"].as<double>();
				args_c.m_height=geom_params["height"].as<double>();

			}
			else if (geom_type == "sphere")
			{
				double radius = geom_params["radius"].as<double>();
				obstacle_list.push_back(create_obstacle(new sphere_t(name,radius,obstacle_pose)));
				obstacle_names.push_back(name);
				shapeType=GEOM_SPHERE;	
				args_v.m_radius=geom_params["radius"].as<double>();
				args_c.m_radius=geom_params["radius"].as<double>();
			}
			else
			{
				prx_throw("Obstacle loader can't load an obstacle of type: "<<geom_type);
			}

			if(sim!=NULL && shapeType!=-1){			  	 
			  args_v.m_shapeType=shapeType;
			  args_c.m_shapeType=shapeType;
			  
			  b3RobotSimulatorCreateMultiBodyArgs args_mb;
			  args_mb.m_baseVisualShapeIndex = sim->createVisualShape(shapeType, args_v);			 
			  args_mb.m_baseCollisionShapeIndex = sim->createCollisionShape(shapeType, args_c);
			  args_mb.m_basePosition=basePosition;			  
			  args_mb.m_baseOrientation=baseOrientation;
			  sim->createMultiBody(args_mb);			  
			}
		}

		return std::make_pair(std::move(obstacle_names),std::move(obstacle_list));
	}
}