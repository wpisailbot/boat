#include <eigen3/Eigen/Core>

using Eigen::Vector3d;
using Eigen::Matrix3d;

// Transforms a vector x using rotation matrix R and origin.
inline Vector3d Trans(const Vector3d& x, const Matrix3d& R, const Vector3d& o) {
  return R * x + o;
}
