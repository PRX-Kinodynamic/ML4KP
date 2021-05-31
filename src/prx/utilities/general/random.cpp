#include "prx/utilities/general/random.hpp"
#include <cmath>
#include <cstdlib>

namespace prx
{

	void init_random(int seed)
	{
		srand(seed);
	}

	double uniform_random()
	{
		double val = rand()*1.0 / RAND_MAX;
		return val;
	}

	double uniform_random(double min, double max)
	{
		// TODO: This implementation is problematic if (max - min) > std::::numeric_limits<double>::max() 
		// ==> change to use std::uniform_real_distribution?
		
		double val = (((double)rand() / (double)RAND_MAX) * (max - min)) + min;
		return val;
	}

	int uniform_int_random(int min, int max)
	{
		int val = (rand() % (max + 1 - min)) + min;
		return val;
	}

	int roll_weighted_die(std::vector<double> const& weights)
	{
		int event_index = -1;

		double sum = 0;
		for( unsigned i = 0; i < weights.size(); i++ )
		{
			sum += weights[i];
		}
		double val = uniform_random();
		double running_total = 0;
		for( unsigned i = 0; i < weights.size(); i++ )
		{
			running_total += weights[i] / sum;
			if( val <= running_total )
			{
				event_index = i;
				break;
			}
		}
		return event_index;
	}

	double gaussian_random()
	{
		static int flag = 0;
		static double t = 0.0;
		double v1,v2,r;
		if (flag == 0) 
		{
			do 
			{
				v1 = 2.0 * uniform_random() - 1.0;
				v2 = 2.0 * uniform_random() - 1.0;
				r = v1 * v1 + v2 * v2;
			} 
			while (r == 0.0 || r > 1.0);
			r = sqrt((-2.0*log(r))/r);
			t = v2*r;
			flag = 1;
			return (v1*r);  
		}
		else 
		{
			flag = 0;
			return t;
		}
	}
}
