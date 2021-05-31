#pragma once

#include "prx/utilities/general/transforms.hpp"

#include <map>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>

namespace prx
{
	#define PRX_PI 3.1415926535897932385
	#define PRX_EPSILON 1e-7
	#define PRX_INFINITY 1e10

	static inline
	std::string lib_path_safe(std::string env_var)
	{
		char* path = std::getenv(env_var.c_str());
		if (path == NULL)
		{
			std::cout << env_var << " environmental variable not set." << std::endl;
			exit(1);
		}
		return std::string(path);
	}

	const std::string lib_path = lib_path_safe("DIRTMP_PATH");
	
	const std::string models_path = lib_path + "resources/models/";
	const std::string input_path = lib_path + "resources/input_files/";
	const std::string js_path = lib_path + "resources/js/";

	enum propagate_step { FIRST_STEP, MIDDLE_STEP, FINAL_STEP };
	enum plant_type { ANALYTICAL, BULLET };

	static inline
	double norm_angle_pi( double angle, double min_angle = -PRX_PI, double max_angle = PRX_PI )
	{
		prx_warn_cond(std::fabs(angle) < 100 * max_angle, "Angle might be too high: " << std::to_string(angle))
		while( angle > max_angle )
			angle -= 2.0 * PRX_PI;
		while( angle < min_angle )
			angle += 2.0 * PRX_PI;
		return angle;
	}

	static inline
	vector_t heatmap_value(double val)
	{
		const std::vector<double> r_vals = {165,215,244,253,254,224,171,116,69,49};
        const std::vector<double> g_vals = {0,48,109,174,224,243,217,173,117,54};
		const std::vector<double> b_vals = {38,39,67,97,144,248,233,209,180,149};

        const double d_index = std::min(val*10.0,10.0);
        const int lower_index = std::min(std::floor(d_index),9.0);
        const int ceil_index = std::min(std::ceil(d_index),9.0);

        return vector_t((r_vals[lower_index]*(1.0-(d_index-lower_index))+r_vals[ceil_index]*((d_index-lower_index)))/255.0,
        				(g_vals[lower_index]*(1.0-(d_index-lower_index))+g_vals[ceil_index]*((d_index-lower_index)))/255.0,
        				(b_vals[lower_index]*(1.0-(d_index-lower_index))+b_vals[ceil_index]*((d_index-lower_index)))/255.0);
	}
	static inline
	vector_t mono_heatmap_value(double val)
	{
		const std::vector<double> r_vals = {0,28.33,56.67,85.0,113.33,141.67,170.0,198.33,226.67,255.};
        const std::vector<double> g_vals = {0,28.33,56.67,85.0,113.33,141.67,170.0,198.33,226.67,255.};
		const std::vector<double> b_vals = {0,28.33,56.67,85.0,113.33,141.67,170.0,198.33,226.67,255.};

        const double d_index = std::min(val*10.0,10.0);
        const int lower_index = std::min(std::floor(d_index),9.0);
        const int ceil_index = std::min(std::ceil(d_index),9.0);

        return vector_t(val, val, val);
	}


	static inline
	std::string rgb_to_string(double r, double g, double b)
	{
		std::stringstream stream;
		stream<<"0x";
		stream << std::setfill('0') << std::setw(2) << std::hex << (int)(r*255);
		stream << std::setfill('0') << std::setw(2) << std::hex << (int)(g*255);
		stream << std::setfill('0') << std::setw(2) << std::hex << (int)(b*255);
		return stream.str();
	}

	static inline
	std::string rgb_to_string(vector_t v)
	{
		return rgb_to_string(v.x(),v.y(),v.z());
	}

	static inline
	vector_t string_to_rgb(std::string input)
	{
		std::vector<std::pair<char,char>> pairs;
		std::vector<double> outputs;
		pairs.push_back(std::make_pair(input[2],input[3]));
		pairs.push_back(std::make_pair(input[4],input[5]));
		pairs.push_back(std::make_pair(input[6],input[7]));
		for(auto val : pairs)
		{
			int value = 0;
			if(val.first>='a')
			{
				value+=((val.first-'a')+10)*16;
			}
			else
			{
				value+=((val.first-'0'))*16;
			}

			if(val.second>='a')
			{
				value+=((val.second-'a')+10);
			}
			else
			{
				value+=((val.second-'0'));
			}
			outputs.push_back(value/255.0);
		}
		return vector_t(outputs[0],outputs[1],outputs[2]);
	}

	template <typename T> std::vector<T> linspace(T a, T b, size_t N)
	{
    	T h = (b - a) / static_cast<T>(N-1);
    	std::vector<T> xs(N);
    	typename std::vector<T>::iterator x;
    	T val;
    	for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        	*x = val;
    	return xs;
	}

	template <typename T> T vector_norm(std::vector<T> a)
	{
		double norm = 0;
		for (int i = 0; i < a.size(); i++)
			norm += a.at(i) * a.at(i);
		return norm;
	}

	template <typename T> int sgn(T a) 
	{
		return (a > 0) - (a < 0);
	}
}
