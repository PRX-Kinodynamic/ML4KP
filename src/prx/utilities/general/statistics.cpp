#include "prx/utilities/general/statistics.hpp"

namespace prx
{		

	void statistics_t::add_sample(std::vector<double>& new_sample)
	{
		samples.push_back(new_sample);
	}
		
	std::vector<double> statistics_t::sample_mean()
	{
		auto fn = [](std::vector<double> accum, std::vector<double> next)
		{
			for (int i = 0; i < next.size(); ++i)
			{
				accum[i] += next[i];
			}
			return accum;
		};
		auto sum = std::accumulate(samples.begin(), samples.end(), std::vector<double>(samples[0].size(), 0.0), fn) ;
		double n = samples.size();
		std::transform(sum.begin(), sum.end(), sum.begin(),
                   [n](double s) -> double { return s / n;  });
		return sum ;
	}

	std::vector<double> statistics_t::sample_variance()
	{
		// @book{montgomery2007introduction,
  			// title={Introduction to statistical quality control},
  			// author={Montgomery, Douglas C},
  			// year={2007},
  			// publisher={John Wiley \& Sons}
  			// page={71}
		// }
		
		const double n = samples.size();

		std::vector<std::vector<double>> sums =
			{std::vector<double>(samples[0].size(), 0.0), std::vector<double>(samples[0].size(), 0.0)} ;
		// auto zipped = zip_iters(sum, sum_squares);
		auto fn = [&](std::vector<std::vector<double>>* accum, std::vector<double> next)
		{
			for (int i = 0; i < next.size(); ++i)
			{
				(*accum)[0][i] += next[i];
				(*accum)[1][i] += std::pow(next[i], 2.0);
			}
			return accum;
		};
		
		auto res = std::accumulate(samples.begin(), samples.end(), &sums, fn) ;

		auto fn2 = [&](std::tuple<std::vector<double>::iterator, std::vector<double>::iterator>& next)
		{
			double s, s_sq;
			std::tie(s, s_sq) = prx::unzip(next);

			return (s_sq - (std::pow(s, 2.0) / n) ) / (n - 1); 
		};
		auto zipped = zip_iters(sums[0], sums[1]);
		std::transform(zipped.begin(), zipped.end(), sums[0].begin(), fn2);
		return sums[0];
	}

	std::vector<double> statistics_t::sample_standard_deviation()
	{	
		auto sv = sample_variance();
		std::transform(sv.begin(), sv.end(), sv.begin(),
                   [](double s) -> double { return std::sqrt(s);  });
		return sv;
	}

	std::string statistics_t::serialize(std::string sep)
	{
		std::stringstream ss;
		auto fn = [&](std::vector<double> data)
		{
			auto last = data.end();
			for (auto first = data.begin(); first != last; )
			{
				ss << *first;
				if (++first != last)
					ss << sep;
			}
			ss << std::endl;
		};
		ss << "Sample_mean"<< sep;
		fn(sample_mean());
		ss << "Sample_variance"<< sep;
		fn(sample_variance());
		ss << "Sample_standard_deviation" << sep;
		fn(sample_standard_deviation());
		return ss.str();
	}

    double statistics_t::hypergeometric_2F1( double a, double b, double c, double x )
	{
		// For more information see: https://mathworld.wolfram.com/HypergeometricFunction.html
		prx_assert(std::fabs(x) < 1.0, "Hypergeometric function - x must be: |x| < 1.0 ");
		double term = a * ( b * ( x / c) );
		double value = 1.0 + term;
		double n = 1;
	
		while ( std::fabs( term ) > PRX_EPSILON )
		{
	   		a++; b++; c++; n++;
	   		term *= a * ( b * ( x / c) ) / n;
	   		value += term;
		}
	
	   return value;
	}


}

