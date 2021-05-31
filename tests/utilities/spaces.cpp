#define BOOST_AUTO_TEST_MAIN spaces_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/spaces/space.hpp"


BOOST_AUTO_TEST_CASE( spaces_test )
{   
    double x,y,theta;
    x = y = theta = 1;
    std::vector<double*> address_1 = {&x, &y, &theta};
    prx::space_t space_1("EER", address_1, "space_1");

    prx::space_point_t pt_sp1 = space_1.make_point();

    space_1.copy_to_point(pt_sp1);

    BOOST_CHECK((*pt_sp1)[0] == pt_sp1 -> at(0));
    BOOST_CHECK((*pt_sp1)[1] == pt_sp1 -> at(1));
    BOOST_CHECK((*pt_sp1)[2] == pt_sp1 -> at(2));
    BOOST_CHECK(pt_sp1 -> get_dim() == space_1.get_dimension());

    // Checking add
    pt_sp1 -> add(pt_sp1);
    BOOST_CHECK((*pt_sp1)[0] == 2);
    BOOST_CHECK((*pt_sp1)[1] == 2);
    BOOST_CHECK((*pt_sp1)[2] == 2);

    // Checking multiply
    pt_sp1 -> multiply(2);
    BOOST_CHECK((*pt_sp1)[0] == 4);
    BOOST_CHECK((*pt_sp1)[1] == 4);
    BOOST_CHECK((*pt_sp1)[2] == 4);

    // Checking add multiply
    pt_sp1 -> add_multiply(2, pt_sp1);
    BOOST_CHECK((*pt_sp1)[0] == 12);
    BOOST_CHECK((*pt_sp1)[1] == 12);
    BOOST_CHECK((*pt_sp1)[2] == 12);

    prx::space_point_t pt_sp2 = space_1.clone_point(pt_sp1);
    BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
    (*pt_sp1)[0] = 10;
    (*pt_sp1)[1] = 20;
    (*pt_sp1)[2] = 3;
    space_1.copy_from_point(pt_sp1);
    space_1.copy_to_point(pt_sp2);
    BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));
    (*pt_sp1)[0] = -10;
    (*pt_sp1)[1] = -20;
    (*pt_sp1)[2] = -3;
    space_1.copy_point(pt_sp2, pt_sp1);
    BOOST_CHECK(space_1.equal_points(pt_sp1, pt_sp2));

    space_1.copy_point_from_vector(pt_sp2, {2,5,1});
    std::vector<double> v;
    space_1.copy_vector_from_point(v, pt_sp2);
    BOOST_CHECK(v[0] == 2 && v[1] == 5 && v[2] == 1);

    (*pt_sp1)[0] = (*pt_sp2)[0] = -100;
    (*pt_sp1)[1] = (*pt_sp2)[1] = -200;
    (*pt_sp1)[2] = (*pt_sp2)[2] = 0;
    BOOST_CHECK(!space_1.satisfies_bounds(pt_sp1));
    space_1.copy_from_point(pt_sp2);
    space_1.set_bounds({-10, -10, -3}, {10, 10, 3});
    space_1.enforce_bounds(pt_sp1);
    BOOST_CHECK(space_1.satisfies_bounds(pt_sp1));
    space_1.enforce_bounds();
    space_1.copy_to_point(pt_sp2);
    BOOST_CHECK(space_1.satisfies_bounds(pt_sp2));
    space_1.sample(pt_sp1);
    BOOST_CHECK(space_1.satisfies_bounds(pt_sp1));

    // integrate test not necessary: testing integration in other test by its own
    // TODO: add interpolate test

    (*pt_sp1)[0] = 0;
    (*pt_sp1)[1] = -1;
    (*pt_sp1)[2] = 2;
    (*pt_sp2)[0] = 1;
    (*pt_sp2)[1] = -2;
    (*pt_sp2)[2] = 3;
    BOOST_CHECK(prx::space_t::l1_norm(pt_sp1) == 3);
    BOOST_CHECK(prx::space_t::l1_norm(pt_sp1, pt_sp2) == 3);
    BOOST_CHECK(prx::space_t::l2_norm(pt_sp1) == std::sqrt(1+4));
    BOOST_CHECK(prx::space_t::l2_norm(pt_sp1, pt_sp2) == std::sqrt(1+1+1));
    (*pt_sp1)[1] = 1;
    (*pt_sp2)[1] = 2;

    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2,1) == 6);
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2,2) == std::sqrt(14));
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2,3) == std::pow(6, 2.0/3.0));
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2,4) == std::pow(2, 1.0/4) * std::sqrt(7));

    (*pt_sp1)[0] = 0;
    (*pt_sp1)[1] = 1;
    (*pt_sp1)[2] = 2;
    (*pt_sp2)[0] = 1;
    (*pt_sp2)[1] = 3;
    (*pt_sp2)[2] = 5;
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1,1) == 6);
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1,2) == std::sqrt(14));
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1,3) == std::pow(6, 2.0/3.0));
    BOOST_CHECK(prx::space_t::lp_norm(pt_sp2, pt_sp1,4) == std::pow(2, 1.0/4) * std::sqrt(7));

    BOOST_CHECK(prx::space_t::euclidean_2d(pt_sp2, pt_sp1) == std::sqrt(1+4));

}