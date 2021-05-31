#include "prx/utilities/geometry/geometry.hpp"

namespace prx
{
	namespace
	{
		struct face_t
		{
			face_t(int i_,int j_,int k_)
			{
				i=i_;
				j=j_;
				k=k_;
			}
			int i,j,k;
		};

		PQP_Model* new_pqp_model(std::vector<std::vector<double>>& vertices,std::vector<face_t>& faces)
		{
            PQP_Model* model = new PQP_Model();

            model->BeginModel();

            for( int i = 0; i < faces.size(); ++i )
            {
            	double first[3],second[3],third[3];
                auto& face = faces[i];

                auto vertex = vertices[face.i];
                first[0] = vertex[0];
                first[1] = vertex[1];
                first[2] = vertex[2];

                vertex = vertices[face.j];
                second[0] = vertex[0];
                second[1] = vertex[1];
                second[2] = vertex[2];

                vertex = vertices[face.k];
                third[0] = vertex[0];
                third[1] = vertex[1];
                third[2] = vertex[2];

                model->AddTri(first, second, third, i);
            }

            model->EndModel();
            return model;
		}

        PQP_Model* create_box_trimesh(double lx, double ly, double lz)
        {
        	std::vector<std::vector<double>> vertices;
        	std::vector<face_t> faces;
            std::vector<double> v={0,0,0};
            for (int i = 0; i < 8; i++)
            {
                v[0] = (lx / 2.0);
                v[1] = (ly / 2.0);
                v[2] = (lz / 2.0);

                if (i % 2 == 1)
                    v[2] = -v[2];
                if (i % 4 >= 2)
                    v[1] = -v[1];
                if (i >= 4)
                    v[0] = -v[0];

                vertices.push_back(v);
            }

            faces.push_back(face_t(0, 1, 2));
            faces.push_back(face_t(1, 3, 2));
            faces.push_back(face_t(4, 0, 6));
            faces.push_back(face_t(0, 2, 6));
            faces.push_back(face_t(5, 4, 7));
            faces.push_back(face_t(4, 6, 7));
            faces.push_back(face_t(1, 5, 3));
            faces.push_back(face_t(5, 7, 3));
            faces.push_back(face_t(5, 1, 4));
            faces.push_back(face_t(1, 0, 4));
            faces.push_back(face_t(7, 6, 2));
            faces.push_back(face_t(3, 7, 2));

            return new_pqp_model(vertices,faces);
        }
        PQP_Model* create_sphere_trimesh(double radius)
        {
        	std::vector<std::vector<double>> vertices;
        	std::vector<face_t> faces;
            std::vector<double> v={0,0,0};
            double angle, height_angle;
            for (int i = 0; i < 31; i++)
            {
                height_angle = (((double) (15 - i)) * PRX_PI / 30.0);
                for (int j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * radius;
                    v[1] = sin(height_angle) * radius;
                    v[2] = sin(angle) * cos(height_angle) * radius;
                    vertices.push_back(v);
                }
            }

            for (int i = 0; i < 31 - 1; i++)
            {
                for (int j = 0; j < 30 - 1; j++)
                {
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + j, (i + 1)*30 + j + 1));
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + (j + 1), i * 30 + (j + 1)));
                }

                faces.push_back(face_t(i * 30 + 29, (i + 1)*30 + 29, (i + 1)*30));
                faces.push_back(face_t(i * 30 + 29, (i + 1)*30, i * 30));
            }
            return new_pqp_model(vertices,faces);
        }


        PQP_Model* create_ellipsoid_trimesh( double a, double b, double c )
        {

        	std::vector<std::vector<double>> vertices;
        	std::vector<face_t> faces;
            std::vector<double> v={0,0,0};
            double angle, height_angle;
            for (int i = 0; i < 31; i++)
            {
                height_angle = (((double) (15 - i)) * PRX_PI / 30.0);
                for (int j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * a;
                    v[1] = sin(height_angle) * b;
                    v[2] = sin(angle) * cos(height_angle) * c;
                    vertices.push_back(v);
                }
            }

            for (int i = 0; i < 31 - 1; i++)
            {
                for (int j = 0; j < 30 - 1; j++)
                {
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + j, (i + 1)*30 + j + 1));
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + (j + 1), i * 30 + (j + 1)));
                }

                faces.push_back(face_t(i * 30 + 29, (i + 1)*30 + 29, (i + 1)*30));
                faces.push_back(face_t(i * 30 + 29, (i + 1)*30, i * 30));
            }

            return new_pqp_model(vertices,faces);
        }

        PQP_Model* create_cone_trimesh(double radius, double height)
        {
            // double points = (int) ((2.0 * PRX_PI * radius / 10.0));
            double angle;
        	std::vector<std::vector<double>> vertices;
        	std::vector<face_t> faces;
            std::vector<double> v={0,0,0};

            vertices.push_back({0.0, 0.0, height});
            vertices.push_back({0.0, 0.0, 0.0});
            vertices.push_back({radius, 0.0, 0.0});

            for (int i = 3; i <= 33; ++i)
            {
                angle = ((double) (i * 2.0 * PRX_PI)) / 30.0;
                v = {cos(angle) * radius, sin(angle) * radius, 0.0};
                vertices.push_back(v);
                faces.push_back(face_t(0, i - 1, i));
                faces.push_back(face_t(1, i, i - 1));
            }
            faces.push_back(face_t(0, 33, 2));
            faces.push_back(face_t(1, 2, 33));
            vertices.push_back({radius*2, 0.0, 0.0});
            faces.push_back(face_t(0, 1, 34));

            return new_pqp_model(vertices,faces);
        }

        PQP_Model* create_cylinder_trimesh(double radius, double height)
        {
            double angle;
        	std::vector<std::vector<double>> vertices;
        	std::vector<face_t> faces;
            std::vector<double> v1={0,0,0};
            std::vector<double> v2={0,0,0};
            for (int i = 0; i < 62; ++i)
                vertices.push_back({0,0,0});

            for (int i = 0; i < 30; i++)
            {
                angle = ((double) (i * 2.0 * PRX_PI)) / 30.0;


                v1[0] = v2[0] = cos(angle) * radius;
                v1[1] = v2[1] = sin(angle) * radius;
                v1[2] = height / 2.0;
                v2[2] = -height / 2.0;

                vertices[i] = v1;
                vertices[31 + i] = v2;
            }

            v1[0] = 0;
            v1[1] = 0;
            v1[2] = height / 2.0;
            vertices[30] = v1;

            v1[0] = 0;
            v1[1] = 0;
            v1[2] = -height / 2.0;
            vertices[61] = v1;

            for (int i = 0; i < 29; i++)
            {
                faces.push_back(face_t(i, i + 31, i + 1));
                faces.push_back(face_t(i + 1, i + 31, i + 1 + 31));
            }
            faces.push_back(face_t(29, 60, 0));
            faces.push_back(face_t(0, 60, 31));


            for (int i = 0; i < 29; i++)
                faces.push_back(face_t(i, i + 1, 30));
            faces.push_back(face_t(29, 0, 30));

            for (int i = 0; i < 29; i++)
                faces.push_back(face_t(i + 31, 61, i + 1 + 31));
            faces.push_back(face_t(60, 61, 31));

            return new_pqp_model(vertices,faces);
        }

        std::pair<std::vector<std::vector<double>>,std::vector<face_t>> create_capsule_info(double radius, double height)
        {            
            double angle, height_angle;
            int i, j;

            std::vector<std::vector<double>> vertices;
            std::vector<face_t> faces;
            std::vector<double> v={0,0,0};

            for (i = 0; i < 16; i++)
            {
                height_angle = (((double) (15 - i)) * PRX_PI / 30.0);
                for (j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * radius;
                    v[1] = sin(angle) * cos(height_angle) * radius;
                    v[2] = sin(height_angle) * radius + height / 2.0;
                    vertices.push_back(v);
                }
            }

            for (i = 16; i < 32; i++)
            {
                height_angle = (((double) (16 - i)) * PRX_PI / 30.0);
                for (j = 0; j < 30; j++)
                {
                    angle = ((double) (j * 2.0 * PRX_PI)) / 30.0;

                    v[0] = cos(angle) * cos(height_angle) * radius;
                    v[1] = sin(angle) * cos(height_angle) * radius;
                    v[2] = sin(height_angle) * radius - height / 2.0;
                    vertices.push_back(v);
                }
            }

            for (int i = 0; i < 32 - 1; i++)
            {
                for (int j = 0; j < 30 - 1; j++)
                {
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + j, (i + 1)*30 + j + 1));
                    faces.push_back(face_t(i * 30 + j, (i + 1)*30 + (j + 1), i * 30 + (j + 1)));

                }

                faces.push_back(face_t(i * 30 + 29, (i + 1)*30 + 29, (i + 1)*30));
                faces.push_back(face_t(i * 30 + 29, (i + 1)*30, i * 30));
            }
            return std::make_pair(std::move(vertices),std::move(faces));
        }

        PQP_Model* create_capsule_trimesh(double radius, double height)
        {
            auto info_vals = create_capsule_info(radius, height);
            return new_pqp_model(info_vals.first,info_vals.second);
        }

	}

	geometry_t::geometry_t(geometry_type_t new_geom_type) : geom_type(new_geom_type),
															params_set(false)
	{
        vis_color = "0xff0000";
	}
	geometry_t::~geometry_t()
	{
	}

	std::weak_ptr<PQP_Model> geometry_t::get_collision_geometry()
	{
		prx_assert(collision_geometry,"Collision geometry has not been generated, but is being requested.");
		return collision_geometry;
	}

	geometry_type_t geometry_t::get_geometry_type()
	{
		return geom_type;
	}

	std::vector<double> geometry_t::get_geometry_params()
	{
		prx_assert(params_set,"Geometry params have not been provided, but are requested to be returned.");
		return params;
	}

	void geometry_t::initialize_geometry(const std::vector<double>& geom_params)
	{
		prx_assert(!params_set,"Trying to call initialize_geometry on the same geometry object twice.");
		params_set = true;
		switch(geom_type)
		{
			case geometry_type_t::BOX:
				prx_assert(geom_params.size()==3,"Trying to initialize a box with "<<geom_params.size()<<" numbers.");
				break;
			case geometry_type_t::SPHERE:
				prx_assert(geom_params.size()==1,"Trying to initialize a sphere with "<<geom_params.size()<<" numbers.");
				break;
			case geometry_type_t::ELLIPSOID:
				prx_assert(geom_params.size()==3,"Trying to initialize an ellipsoid with "<<geom_params.size()<<" numbers.");
				break;
			case geometry_type_t::CAPSULE:
				prx_assert(geom_params.size()==2,"Trying to initialize a capsule with "<<geom_params.size()<<" numbers.");
				break;
			case geometry_type_t::CONE:
				prx_assert(geom_params.size()==2,"Trying to initialize a cone with "<<geom_params.size()<<" numbers.");
				break;
			case geometry_type_t::CYLINDER:
				prx_assert(geom_params.size()==2,"Trying to initialize a cylinder with "<<geom_params.size()<<" numbers.");
				break;
		};
		params = geom_params;
	}

	void geometry_t::generate_collision_geometry()
	{
		prx_assert(params_set,"Geometry params have not been provided, so a collision geometry cannot be generated.");
		prx_assert(collision_geometry==nullptr,"Trying to recreate collision geometries when they have already been created.");
		switch(geom_type)
		{
			case geometry_type_t::BOX:
				collision_geometry = std::shared_ptr<PQP_Model>(create_box_trimesh(params[0],params[1],params[2]));
				break;
			case geometry_type_t::SPHERE:
				collision_geometry = std::shared_ptr<PQP_Model>(create_sphere_trimesh(params[0]));
				break;
			case geometry_type_t::ELLIPSOID:
				collision_geometry = std::shared_ptr<PQP_Model>(create_ellipsoid_trimesh(params[0],params[1],params[2]));
				break;
			case geometry_type_t::CAPSULE:
				collision_geometry = std::shared_ptr<PQP_Model>(create_capsule_trimesh(params[0],params[1]));
				break;
			case geometry_type_t::CONE:
				collision_geometry = std::shared_ptr<PQP_Model>(create_cone_trimesh(params[0],params[1]));
				break;
			case geometry_type_t::CYLINDER:
				collision_geometry = std::shared_ptr<PQP_Model>(create_cylinder_trimesh(params[0],params[1]));
				break;
		};
	}
}