#define BOOST_AUTO_TEST_MAIN statistics_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/general/statistics.hpp"


BOOST_AUTO_TEST_CASE( statistics_test )
{   
    prx::statistics_t stats;

    double n = 1000;
    for (double i = 0; i <= n; ++i)
    {
    	std::vector<double> v = {i,i - n / 2.};
    	stats.add_sample(v);
    }

    auto sm = stats.sample_mean();
    // std::cout << "sm[0]: " << sm[0] << std::endl; 
    // std::cout << "sm[1]: " << sm[1] << std::endl; 
    // Sum 1 to n = n * (n+1) / 2.
    BOOST_CHECK(sm[0] == (n * (n + 1) / 2.) / (n + 1)); // dividing over n+1 because of '<=' on for
    BOOST_CHECK(sm[1] == 0.0);

    auto sv = stats.sample_variance();
    auto ssd = stats.sample_standard_deviation();
    // std::cout << "sv[0]: " << sv[0] << std::endl; 
    // std::cout << "sv[1]: " << sv[1] << std::endl; 
    BOOST_CHECK(sv[0] == 83583.5); // Result for [0...1000]
    BOOST_CHECK(sv[1] == 83583.5); // Result for [0...1000]
    BOOST_CHECK(ssd[0] == ssd[1]); // Result for [0...1000]

    const double TOLERANCE = 1e-4;
    // std::cout << "hypergeometric_2F1: " << prx::statistics_t::hypergeometric_2F1(2,3,4,0.5) << std::endl;
    // std::cout << "hypergeometric_2F1: " << std::setprecision(10) << std::fixed << prx::statistics_t::hypergeometric_2F1(2,3,4,0.9) << std::endl;

    BOOST_CHECK(std::fabs(prx::statistics_t::hypergeometric_2F1(2,3,4,0.5) - 2.7289353331 ) < TOLERANCE );
    BOOST_CHECK(std::fabs(prx::statistics_t::hypergeometric_2F1(2,3,4,0.1) - 1.1702393863 ) < TOLERANCE );
    BOOST_CHECK(std::fabs(prx::statistics_t::hypergeometric_2F1(2,3,4,0.9) - 21.789423102929 ) < TOLERANCE );
    
    // std::cout << "hypergeometric_2F1_BETA: " << prx::statistics_t::hypergeometric_2F1(5,1-10, 5+1, 0.1) << std::endl;
    // std::cout << "incomplete_beta: " << prx::statistics_t::incomplete_beta(0.1,5,10) << std::endl;
    // BOOST_CHECK(std::fabs(prx::statistics_t::incomplete_beta(0.1,5,10) - 9.22099146438E-7 ) < TOLERANCE );
    // BOOST_CHECK(std::fabs(prx::statistics_t::incomplete_beta(0.5,5,10) - 0.00009093079 ) < TOLERANCE );
    // BOOST_CHECK(std::fabs(prx::statistics_t::incomplete_beta(0.9,5,10) - 0.00009990009 ) < TOLERANCE );

    // BOOST_CHECK(std::fabs(prx::statistics_t::beta(5,10) - 0.0000999000999 ) < TOLERANCE );
    // BOOST_CHECK(std::fabs(prx::statistics_t::beta(50,100) - 1.4904121110955478E-42 ) < TOLERANCE );
    // BOOST_CHECK(std::fabs(prx::statistics_t::beta(1,10) - 0.1 ) < TOLERANCE );


}