/**
 * @file gripper_eigen_typedefs.hpp
 * @brief Contains Eigen typedefs used in this package.
 */

#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_EIGEN_TYPEDEFS_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_EIGEN_TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>

namespace Eigen {

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 2> Matrix6x2d;
typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

}  // namespace Eigen

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_EIGEN_TYPEDEFS_HPP_
