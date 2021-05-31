/**
 * @file random.hpp
 * @brief <b> A few helper functions for random sampling. </b>
 * */
#pragma once

#include <vector>

namespace prx
{

    /**
    * Initializes the uniform random number generator with the given seed.
    * 
    * @brief Initializes the uniform random number generator with the given seed.
    * @param seed The value to seed the RNG with.  Default is 10.
    * @author Zakary Littlefield
    */
    void init_random(int seed);

    /**
    * Returns a random number from the uniform distribution [0,1).
    *
    * @brief Returns a random number from the uniform distribution [0,1).
    * @author Zakary Littlefield
    * 
    * @return A double precision random number.
    */
    double uniform_random();


    /**
    * Returns a random number from a Gaussian Distribution
    *
    * @brief Returns a random number from a Gaussian Distribution.
    * @author Zakary Littlefield
    * @return A double precision random number.
    */
    double gaussian_random();

    /**
    * Returns a random number from the uniform distribution within the
    * given range.
    * 
    * @brief Returns a random number from the uniform distribution within the given range.
    * @param min The minimum random value to return.
    * @param max The maximum random value to return.
    * @author Zakary Littlefield
    *
    * @return A double precision random number in the given range.
    */
    double uniform_random(double min, double max);

    /**
    * Returns a random integer number from the uniform distribution within
    * the given closed range [min,max]. Both max and min are possible return values
    *
    * @brief Returns a random integer number from the uniform distribution within the given closed range.
    * @param min The minimum random value to return.
    * @param max The maximum random value to return.
    * @author Zakary Littlefield
    *
    * @return An integer random number in the given range.
    */
    int uniform_int_random(int min, int max);

    /**
    * Given a set of weights, randomly roll a "dice" and obtain an event
    * 
    * For example, given a probability distribution {10%,10%,80%} this function
    * will roll a dice weighted with those probabilities and return the index
    * of the event that occurred.
    * 
    * @brief Given a set of weights, randomly roll a "dice" and obtain an event
    * @param weights A probability distribution signifying how likely an event will occur
    * @authors Andrew Kimmel, Nick Stiffler
    * 
    * @return The index of the weighted event that happened
    */
    int roll_weighted_die(std::vector<double> const& weights);

}