#include "prx/utilities/general/timer.hpp"

#include <cmath>
#include <cstddef>

namespace prx
{
    timer_t::timer_t()
    {
        reset();
        elapsed = 0;
    }

    timer_t::~timer_t()
    {}

    double timer_t::get_time_in_secs()
    {  
        gettimeofday( &finish, NULL ); 
        double s = finish.tv_sec;
        s += ( 0.000001 * finish.tv_usec );
        return s;
    }

    void timer_t::reset()
    {
        gettimeofday( &start, NULL );
    }


    double timer_t::measure()
    {
        gettimeofday( &finish, NULL ); 
        elapsed = (finish.tv_sec - start.tv_sec) + 0.000001 * (finish.tv_usec - start.tv_usec);
        return elapsed;
    }


    double timer_t::measure_reset()
    {
        measure();
        reset();
        return elapsed;
    }

    void timer_t::add_delay_user_clock( double delay )
    {
        int dl = (int)delay;
        int rest = (int)round( (double)(delay - dl) * 1000000.0);
        
        start.tv_sec  -= dl;
        start.tv_usec -= rest;
        if( start.tv_usec < 0 )
        {
    	start.tv_sec -= 1;
    	start.tv_usec = 1000000 + start.tv_usec;
        }
    }
}