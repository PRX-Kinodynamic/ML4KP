#include "prx/utilities/general/random.hpp"

double uniform_random_0()
{
	return prx::uniform_random();
}
double uniform_random_2(double min, double max)
{
	return prx::uniform_random(min, max);
}

void pyprx_utilities_general_random_py()
{

	def("init_random", &prx::init_random);
	def("uniform_random", uniform_random_0);
	def("gaussian_random", &prx::gaussian_random);
	def("uniform_random", uniform_random_2);
	def("uniform_int_random", &prx::uniform_int_random);
	def("roll_weighted_die", &prx::roll_weighted_die);

}