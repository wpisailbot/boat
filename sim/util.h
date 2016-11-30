#pragma once
#include <eigen3/Eigen/Core>
#include "sim/sim_debug.pb.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

constexpr double PI = 3.14159265358979323846264338;

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
  Rn.row(2) = R.row(2) - err/2 * R.row(1);
  Rn.row(1) = R.row(1) - err/2 * R.row(2);
  Rn.row(0) = Rn.row(2).cross(Rn.row(1));
  Rn.row(0) *= .5 * (3 - Rn.row(0).squaredNorm());
  Rn.row(1) *= .5 * (3 - Rn.row(1).squaredNorm());
  Rn.row(2) *= .5 * (3 - Rn.row(2).squaredNorm());
  return Rn;
}

inline void EigenToProto(const Eigen::Vector3d &ve,
                         sailbot::msg::Vector3f *msg) {
  msg->set_x(ve(0, 0));
  msg->set_y(ve(1, 0));
  msg->set_z(ve(2, 0));
}

inline float norm_angle(float a) {
  while (a > M_PI)
    a -= 2 * M_PI;
  while (a < -M_PI)
    a += 2 * M_PI;
  return a;
}
