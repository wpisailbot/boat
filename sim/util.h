#pragma once
#include <eigen3/Eigen/Core>

using Eigen::Vector3d;
using Eigen::Matrix3d;

// Transforms a vector x using rotation matrix R and origin.
inline Vector3d Trans(const Vector3d& x, const Matrix3d& R, const Vector3d& o) {
  return R * x + o;
}

inline Matrix3d Skew(const Vector3d& x) {
  Matrix3d Sx;
  Sx << 0, -x(2), x(1),
        x(2), 0, -x(0),
        -x(1), x(0), 0;
  return Sx;
}

inline Matrix3d Orthogonalize(const Matrix3d&R) {
  double err = R.row(0).dot(R.row(1));
  Matrix3d Rn;
  Rn.row(0) = R.row(0) - err/2 * R.row(1);
  Rn.row(1) = R.row(1) - err/2 * R.row(0);
  Rn.row(2) = Rn.row(0).cross(Rn.row(1));
  Rn.row(0) *= .5 * (3 - Rn.row(0).squaredNorm());
  Rn.row(1) *= .5 * (3 - Rn.row(1).squaredNorm());
  Rn.row(2) *= .5 * (3 - Rn.row(2).squaredNorm());
  return Rn;
}
