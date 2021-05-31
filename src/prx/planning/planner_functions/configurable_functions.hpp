#pragma once

#include "prx/planning/planner_functions/planner_functions.hpp"

namespace prx
{
	class configurable_functions_t : public planner_functions_t
	{
	public:

		configurable_functions_t(std::string context_name, world_model_context context, param_loader pl);
	private:
		// Placing some helper functions here
		double bm_metric_cost(std::vector<double> s1)
		{
			if (s1.at(2) < 0)
			{
				s1.at(1) = -s1.at(1); s1.at(2) = -s1.at(2);
				return bm_metric_cost(s1);
			}

			double r = std::sqrt(s1.at(0)*s1.at(0)+s1.at(1)*s1.at(1));
			double zeta = std::atan2(s1.at(1),s1.at(0));

			if ((zeta > (s1.at(2) + PRX_PI)/2 && zeta < PRX_PI) || (zeta > (s1.at(2) - PRX_PI)/2 && zeta < 0))
			{
				double x = s1.at(0); double y = s1.at(1);
				s1.at(0) = x * std::cos(s1.at(2)) + y * std::sin(s1.at(2));
				s1.at(1) = x * std::sin(s1.at(2)) - y * std::cos(s1.at(2));
				return bm_metric_cost(s1);
			}

			if (s1.at(1) < 0)
			{
				s1.at(0) = -s1.at(0);
				s1.at(1) = -s1.at(1);
				return bm_metric_cost(s1);
			}

			if (zeta <= s1.at(2))
			{
				return r + std::min(std::fabs(zeta)+std::fabs(zeta - s1.at(2)),2*PRX_PI+std::fabs(zeta)-std::fabs(zeta+s1.at(2)));
			}
			else if (s1.at(1) <= 1 - std::cos(s1.at(2)))
			{
				if (s1.at(1) == 0) return std::fabs(s1.at(0)) + 0.5*s1.at(2);
				else return (s1.at(1) * (1 + std::cos(s1.at(2)))/std::sin(s1.at(2)) - s1.at(0)) + 0.5*s1.at(2);
			}
			else if (r >= std::tan(0.5*zeta))
			{
				return r + std::min(std::fabs(zeta)+std::fabs(zeta - s1.at(2)),2*PRX_PI+std::fabs(zeta)-std::fabs(zeta+s1.at(2)));
			}
			else
			{
				return std::acos(1 - s1.at(1)) - 0.5*s1.at(2) - s1.at(0) + std::sqrt(s1.at(1)*(1-s1.at(1)));
			}
		}
	};
}
