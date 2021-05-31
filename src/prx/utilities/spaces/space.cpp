#include "prx/utilities/spaces/space.hpp"

#include <limits>
#include <sstream>
#include <iomanip>


namespace prx
{

	space_t::space_t(const std::string& topo, const std::vector<double*>& ads, const std::string& name)
	{
		owned_values = true;
		addresses.clear();
		for(double* v: ads)
		{
			addresses.push_back(v);
		}
		dimension = addresses.size();
		prx_assert(topo.size()==addresses.size(),
			"Topology "<<topo<<" does not match addresses size "<<addresses.size());

		topology.clear();
		lower_bounds.clear();
		upper_bounds.clear();
		for(auto c : topo)
		{
			switch(c)
			{
				case 'E':
					topology.push_back(topology_t::EUCLIDEAN);
					lower_bounds.push_back(new double(std::numeric_limits<double>::min()));
					upper_bounds.push_back(new double(std::numeric_limits<double>::max()));
					break;
				case 'R':
					topology.push_back(topology_t::ROTATIONAL);
					lower_bounds.push_back(new double(-PRX_PI));
					upper_bounds.push_back(new double(PRX_PI));
					break;
				case 'D':
					topology.push_back(topology_t::DISCRETE);
					lower_bounds.push_back(new double(0));
					upper_bounds.push_back(new double(std::numeric_limits<int>::max()));
					break;
				default:
					prx_throw("Bad topology identifier '"<<c<<"' from topology string "<<topo);
			}
		}
		space_name = name;
	}

	space_t::space_t(const std::string& topology, const std::vector<double*>& addresses) : space_t(topology,addresses,topology)
	{
	}

	space_t::space_t(const std::vector<const space_t*>& spaces)
	{
		owned_values = false;
		dimension = 0;
		addresses.clear();
		topology.clear();
		space_name="";

		for(auto space : spaces)
		{
			if(space!=nullptr && space->addresses.size()!=0)
			{
				for(auto val : space->addresses)
				{
					addresses.push_back(val);
				}
				for(auto val : space->topology)
				{
					topology.push_back(val);
				}
				for(auto val : space->lower_bounds)
				{
					lower_bounds.push_back(val);
				}
				for(auto val : space->upper_bounds)
				{
					upper_bounds.push_back(val);
				}
				if( !space_name.empty() )
					space_name += "|";
				space_name += space->space_name;
			}
		}
		dimension = addresses.size();

	}

	space_t::~space_t()
	{
		if(owned_values)
		{
			for(auto d : lower_bounds)
			{
				delete d;
			}
			for(auto d : upper_bounds)
			{
				delete d;
			}
		}
	}
	void space_t::set_bounds(const std::vector<double>& lower,const std::vector<double>& upper)
	{
		prx_assert(lower.size()==lower_bounds.size(),"Given lower bounds have size "<<lower.size()<<" while the space bounds have size "<<lower_bounds.size());
		prx_assert(upper.size()==upper_bounds.size(),"Given upper bounds have size "<<upper.size()<<" while the space bounds have size "<<upper_bounds.size());

		for(unsigned i=0;i<dimension;++i)
		{
			prx_assert(lower[i]<=upper[i],"Index "<<i<<" has lower bound ("<<lower[i]<<") greater than upper bound("<<upper[i]<<")");
			*(lower_bounds[i]) = lower[i];
			*(upper_bounds[i]) = upper[i];
		}
	}

	space_point_t space_t::make_point() const
	{
		return std::make_shared<space_snapshot_t>(this,dimension);
	}

	space_point_t space_t::clone_point(const space_point_t& point) const
	{
		prx_assert(point->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		auto new_point = make_point();
		copy_point(new_point,point);
		return new_point;
	}

	void space_t::point_union(const space_point_t& p1, const space_point_t& p2, const space_point_t& pu)
	{
		prx_assert( ( p1 -> parent -> space_name + "|" + p2 -> parent ->space_name )==space_name,
			"Points and space have different names: "<< p1->parent->space_name << "|" << 
			p2 ->parent -> space_name <<" != "<< space_name);
		prx_assert( p1 -> get_dim() + p2 -> get_dim() == dimension, 
			"Points dimension must add to this space dimension: " << p1 -> get_dim() << " + " << p2 -> get_dim()
			<< " != " << dimension );

		// auto new_point = make_point();

		for(unsigned i=0;i<p1 -> get_dim();++i)
		{
			pu->memory[i]=p1->memory[i];
		}

		for (unsigned i = 0; i < p2 -> get_dim() ; ++i)
		{
			pu -> memory[ p1 -> get_dim() + i] = p2 -> memory[i];
		}
		// return new_point;
	}

	void space_t::split_point(const space_point_t& ps_to_split, 
		const space_point_t& ps1, const space_point_t& ps2)
	{
		prx_assert(  (ps1 -> parent -> space_name + "|" + ps2 -> parent -> space_name) ==
			ps_to_split -> parent -> space_name,
			"Different spaces: " << ps1 -> parent -> space_name + "|" + ps2 -> parent -> space_name 
			<< "!=" << ps_to_split -> parent -> space_name  );

		for (int i = 0; i < ps1 -> get_dim(); ++i)
		{
			ps1 -> memory[i] = ps_to_split -> memory[i];
		}

		for (int i = 0; i < ps2 -> get_dim(); ++i)
		{
			ps2 -> memory[i] = ps_to_split -> memory[ ps1 -> get_dim() + i ];
		}
	}

	bool space_t::equal_points(const space_point_t& point1,const space_point_t& point2)
	{
		prx_assert(point1->parent->space_name==point2->parent->space_name,"Points have different parent spaces: "<<point1->parent->space_name<<" and "<<point2->parent->space_name);
		prx_assert(point1->parent->space_name==space_name,"Points and space have different names: "<<point1->parent->space_name<<" and "<<space_name);
		for(unsigned i=0;i<dimension;++i)
		{
			if(fabs(point1->memory[i]-point2->memory[i])>PRX_EPSILON)
			{
				return false;
			}
		}
		return true;
	}

	// void point_addition(const space_point_t& pt1, const space_point_t& pt2, const space_point_t& pt_res)
	// {
	// 	prx_assert(pt1->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
	// 	prx_assert(pt2->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
	// 	prx_assert(pt_res->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		
	// 	for(unsigned i=0;i<dimension;++i)
	// 	{
	// 		pt_res -> memory[i] = pt1 -> memory
	// 	}
	// }


	void space_t::copy_to_point(const space_point_t& point) const
	{
		prx_assert(point->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		for(unsigned i=0;i<dimension;++i)
		{
			point->memory[i]=*addresses[i];
		}
	}
	void space_t::copy_from_point(const space_point_t& point) const
	{
		//NOTE: if this point come from a different space with the same name, different bounds may be in effect. Consider adding bounds enforcement here.
		prx_assert(point->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		for(unsigned i=0;i<dimension;++i)
		{
			*addresses[i]=point->memory[i];
		}
	}

	void space_t::copy_point(const space_point_t& destination,const space_point_t& source) const
	{
		prx_assert(destination->parent->space_name==source->parent->space_name,
			"Points have different parent spaces: "<<destination->parent->space_name<<" and "<<source->parent->space_name);
		prx_assert(destination->parent->space_name==space_name,
			"Points and space have different names: "<<destination->parent->space_name<<" and "<<space_name);
		for(unsigned i=0;i<dimension;++i)
		{
			destination->memory[i]=source->memory[i];
		}

	}
	void space_t::copy_point_from_vector(const space_point_t& destination, const std::vector<double>& source) const
	{
		prx_assert(destination->parent->space_name==space_name,"Point and space have different names: "<<destination->parent->space_name<<" and "<<space_name);
		prx_assert(destination->parent->dimension==source.size(),"Point and vector have different sizes: "<<destination->parent->dimension<<" and "<<source.size());
		for(unsigned i=0;i<dimension;++i)
		{
			destination->memory[i]=source[i];
		}
		enforce_bounds(destination);

	}
	void space_t::copy_vector_from_point(std::vector<double>& destination, const space_point_t& source) const
	{
		prx_assert(source->parent->space_name==space_name,"Point and space have different names: "<<source->parent->space_name<<" and "<<space_name);
		//prx_assert(source->parent->dimension==destination.size(),"Point and vector have different sizes: "<<source->parent->dimension<<" and "<<destination.size());
			// printf("%s:%d\n",__PRETTY_FUNCTION__, __LINE__ );
			// printf("source dim: %d\n", source->parent->dimension);
			// printf("dest dim: %d\n", destination.size());
		for(unsigned i=0;i<dimension;++i)
		{
			// printf("%.4f\n",source -> memory[i]);
			//destination[i]=source->memory[i];
			destination.push_back(source->memory[i]);
			// source->memory[i];
			// printf("...!\n");
			// destination.push_back(15);
			// printf("Done!\n");
		}
	}
	void space_t::enforce_bounds(const space_point_t& point) const
	{
		prx_assert(point->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		for(unsigned i=0;i<dimension;i++)
		{
			double& p = point->memory[i];
			if(topology[i]==topology_t::ROTATIONAL)
			{
        		p = norm_angle_pi(p, *lower_bounds[i], *upper_bounds[i]);
			}
			if(p<*lower_bounds[i])
			{
				p=*lower_bounds[i];
			}
			if(p>*upper_bounds[i])
			{
				p=*upper_bounds[i];
			}
		}
	}
	void space_t::enforce_bounds() const
	{
		for(unsigned i=0;i<dimension;i++)
		{
			double& p = (*addresses[i]);
			if(topology[i]==topology_t::ROTATIONAL)
			{
				p = norm_angle_pi(p);
			}
			if(p<*lower_bounds[i])
			{
				p=*lower_bounds[i];
			}
			if(p>*upper_bounds[i])
			{
				p=*upper_bounds[i];
			}
		}
	}
	bool space_t::satisfies_bounds(const space_point_t& point) const
	{
		prx_assert(point->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		// printf("point: (%.3f, %.3f, %.3f)\n", 
				// point -> at(0), point -> at(1), point -> at(2));
		for(unsigned i=0;i<dimension;i++)
		{
			double& p = point->memory[i];
			if(p<*lower_bounds[i])
			{
				// printf("%d - lower bound: %.3f, val: %.3f\n", i, *lower_bounds[i], p );
				return false;
			}
			if(p>*upper_bounds[i])
			{
				// printf("%d - upper bound: %.3f, val: %.3f\n", i, *lower_bounds[i], p );
				return false;
			}
		}
		return true;
	}
	void space_t::sample(const space_point_t& point) const
	{
		prx_assert(point->parent->space_name==space_name,
			"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);
		for(unsigned i=0;i<dimension;i++)
		{
			point->memory[i] = uniform_random(*lower_bounds[i],*upper_bounds[i]);
			if(topology[i]==topology_t::DISCRETE)
				point->memory[i] = round(point->memory[i]);
		}
	}

	std::vector<std::pair<double,double>> space_t::get_bounds() const
	{
		std::vector<std::pair<double,double>> bounds;
		for(unsigned i=0;i<dimension;i++)
		{
			bounds.push_back(std::make_pair(*lower_bounds[i],*upper_bounds[i]));
		}
		return bounds;
	}

	void space_t::integrate(const space_point_t& point,const space_t* derivative,double delta_t)
	{
		prx_assert(derivative->get_dimension()==dimension,"");
		copy_from_point(point);
		integrate(derivative,delta_t);
	}

	void space_t::integrate(const space_t* derivative,double delta_t)
	{
		for(unsigned i=0;i<dimension;i++)
		{
			if( topology[i] == topology_t::EUCLIDEAN || topology[i] == topology_t::ROTATIONAL )
			{
				*(addresses[i]) += delta_t * derivative->at(i);
			}
			else if(topology[i]==topology_t::DISCRETE)
			{
				*(addresses[i]) += delta_t * derivative->at(i);
				*(addresses[i]) = round(*(addresses[i]));
			}
		}
		enforce_bounds();
	}


	void space_t::interpolate(const space_point_t& point1, const space_point_t& point2, double t, space_point_t& result) const
	{
		prx_assert(point1->parent->space_name==space_name,"Point and space have different names: "<<point1->parent->space_name<<" and "<<space_name);
		prx_assert(point2->parent->space_name==space_name,"Point and space have different names: "<<point2->parent->space_name<<" and "<<space_name);
		prx_assert(result->parent->space_name==space_name,"Point and space have different names: "<<result->parent->space_name<<" and "<<space_name);
		prx_assert(t>=0 && t<=1,"Interpolation requires a value between 0 and 1: given "<<t);

		for(unsigned i=0;i<dimension;i++)
		{
			if(topology[i]==topology_t::ROTATIONAL)
			{
				if(std::fabs(point1->memory[i]-point2->memory[i])<PRX_PI)
				{
					result->memory[i]=(1-t)*point1->memory[i] + t*point2->memory[i];
				}
				else
				{
					if(point1->memory[i]<point2->memory[i])
						result->memory[i] = point2->memory[i] + (1-t)*(point1->memory[i] - point2->memory[i] + 2*PRX_PI);
					else
						result->memory[i] = point1->memory[i] + t*(2*PRX_PI - point1->memory[i]+point2->memory[i]);
				}
				result->memory[i] = norm_angle_pi(result->memory[i]);
			}
			else if(topology[i]==topology_t::DISCRETE)
			{
				if(t==1)
					result->memory[i] = (int)point2->memory[i];
				else
					result->memory[i] = (int)point1->memory[i];
			}
			else
				result->memory[i] = (1-t)*point1->memory[i] + t*point2->memory[i];
		}
	}

	std::string space_t::print_point(const space_point_t& point, unsigned prec) const
	{
		prx_assert(point->parent->space_name==space_name,"Point and space have different names: "<<point->parent->space_name<<" and "<<space_name);

		std::stringstream out(std::stringstream::out);
		if( dimension > 0 )
		{
			// out << std::fixed << std::setprecision(prec) << '<';
			out << std::fixed << std::setprecision(prec);
			for( unsigned i = 0; i < dimension - 1; ++i )
				out << point->memory[i] << ',';
			// out << point->memory[dimension - 1] << '>';
			out << point->memory[dimension - 1];
		}

		return out.str();
	}

	std::string space_t::print_memory(unsigned prec) const
	{
		std::stringstream out(std::stringstream::out);
		if( dimension > 0 )
		{
			out << std::fixed << std::setprecision(prec) << '<';
			for( unsigned i = 0; i < dimension - 1; ++i )
				out << *addresses[i] << ',';
			out << *addresses[dimension - 1] << '>';
		}

		return out.str();
	}

}