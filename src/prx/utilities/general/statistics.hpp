#pragma once

#include <vector>
#include <numeric>
#include "prx/utilities/general/zipped_iter.hpp"

namespace prx
{
	class statistics_t
	{
	public:
		statistics_t() = default;
		~statistics_t() = default;

		void add_sample(std::vector<double>& new_sample);
		
		std::vector<double> sample_mean();

		std::vector<double> sample_variance();

		std::vector<double> sample_standard_deviation();

		// TODO: Implement the following functions
		/**
		 * f test to determine if the null hypothesis (variance of the samples is equal) and that any variation is down to chance. Can also test the alternative hypothesis that any difference is not down to chance.
		 * @param  sd1   Sample 1 variance
		 * @param  sd2   Sample 2 variance
		 * @param  N1    Sample 1 size
		 * @param  N2    Sample 2 size
		 * @param  alpha Significance level
		 * @return       True ==> Not Rejected.
		 */
		// static bool f_test(double v1, double v2, double N1, double N2, double alpha);

		// static double beta(double a, double b);

  //   	static double regularized_beta(double z, double a, double b);

  //   	static double incomplete_beta(double z, double a, double b);

    	static double hypergeometric_2F1( double a, double b, double c, double x );

		std::string serialize(std::string sep = ",");

	protected:
		std::vector<std::vector<double>> samples;

	};
}
