#pragma once

/**
 * @file transforms.hpp
 * @author Zakary Littlefield
 * @brief <b> [INCOMPLETE] Some handy aliases for commonly used Eigen declarations. </b>
 * */

#include <Eigen/Dense>
#include <Eigen/Core>

namespace prx
{
	/** @brief A column vector of dimension <i>N</i>. */
	template <int N>
	using n_vector_t = Eigen::Matrix<double, N, 1>;
	/** @brief A matrix of dimension <i>N</i> x <i>N</i>. */
	template <int N>
	using n_matrix_t = Eigen::Matrix<double, N, N>;
	/** @brief A column vector of dimension 3. */
	using vector_t = n_vector_t<3>;
	/** @brief A 3x3 matrix. */
	using matrix_t = n_matrix_t<3>;
	/** @brief A quaternion. */
	using quaternion_t = Eigen::Quaternion<double>;
	using axis_angle_t = Eigen::AngleAxis<double>;
	using transform_t = Eigen::Transform<double, 3, Eigen::AffineCompact>;
}